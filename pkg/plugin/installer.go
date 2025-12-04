package plugin

import (
	"archive/tar"
	"compress/gzip"
	"context"
	"crypto/sha256"
	"encoding/hex"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"runtime"
	"strings"

	getter "github.com/hashicorp/go-getter"
	"github.com/hashicorp/go-hclog"
)

// BinaryConfig 通用二进制配置
type BinaryConfig struct {
	// BinaryPath 二进制文件路径（如果已安装）
	BinaryPath string `json:"binaryPath,omitempty" yaml:"binaryPath,omitempty"`

	// Installation 安装配置（支持自动下载安装）
	Installation *InstallationConfig `json:"installation,omitempty" yaml:"installation,omitempty"`
}

// InstallationConfig 通用安装配置
type InstallationConfig struct {
	// Enabled 是否启用自动安装
	Enabled bool `json:"enabled,omitempty" yaml:"enabled,omitempty"`

	// Version 版本（如 "1.9.0"）
	Version string `json:"version,omitempty" yaml:"version,omitempty"`

	// DownloadURL 下载地址
	// 支持 go-getter 所有格式：http://, https://, git::, s3::, gcs:: 等
	// 支持变量替换: {{.Version}}, {{.OS}}, {{.Arch}}
	DownloadURL string `json:"downloadUrl,omitempty" yaml:"downloadUrl,omitempty"`

	// InstallDir 安装目录
	InstallDir string `json:"installDir,omitempty" yaml:"installDir,omitempty"`

	// BinaryName 二进制文件名（相对于 InstallDir）
	// 例如: "bin/spire-agent", "kubectl"
	BinaryName string `json:"binaryName,omitempty" yaml:"binaryName,omitempty"`

	// ChecksumURL 校验和文件 URL（可选）
	ChecksumURL string `json:"checksumUrl,omitempty" yaml:"checksumUrl,omitempty"`

	// ChecksumType 校验和类型: sha256, sha512, md5
	ChecksumType string `json:"checksumType,omitempty" yaml:"checksumType,omitempty"`

	// AutoUpdate 是否自动更新到指定版本
	AutoUpdate bool `json:"autoUpdate,omitempty" yaml:"autoUpdate,omitempty"`

	// ExtractArchive 是否需要解压（tar.gz, zip 等）
	ExtractArchive bool `json:"extractArchive,omitempty" yaml:"extractArchive,omitempty"`

	// StripComponents 解压时去除顶层目录层数（类似 tar --strip-components）
	StripComponents int `json:"stripComponents,omitempty" yaml:"stripComponents,omitempty"`
}

// BinaryInstaller 通用二进制安装器
type BinaryInstaller struct {
	logger hclog.Logger
	config *InstallationConfig
}

// NewBinaryInstaller 创建安装器
func NewBinaryInstaller(logger hclog.Logger, config *InstallationConfig) *BinaryInstaller {
	return &BinaryInstaller{
		logger: logger,
		config: config,
	}
}

// EnsureBinary 确保二进制文件存在（必要时安装）
func (i *BinaryInstaller) EnsureBinary(ctx context.Context, binaryConfig *BinaryConfig, fallbackPaths []string) (string, error) {
	// 1. 如果配置了安装，尝试安装
	if binaryConfig != nil && binaryConfig.Installation != nil && binaryConfig.Installation.Enabled {
		installer := NewBinaryInstaller(i.logger, binaryConfig.Installation)
		if err := installer.Install(ctx); err != nil {
			i.logger.Warn("failed to install binary, will try to find existing", "error", err)
		} else {
			binaryPath := installer.GetBinaryPath()
			if _, err := os.Stat(binaryPath); err == nil {
				i.logger.Info("binary installed successfully", "path", binaryPath)
				return binaryPath, nil
			}
		}
	}

	// 2. 如果配置中指定了路径，直接使用
	if binaryConfig != nil && binaryConfig.BinaryPath != "" {
		if _, err := os.Stat(binaryConfig.BinaryPath); err == nil {
			return binaryConfig.BinaryPath, nil
		}
		return "", fmt.Errorf("specified binary not found: %s", binaryConfig.BinaryPath)
	}

	// 3. 尝试 fallback 路径
	for _, path := range fallbackPaths {
		if _, err := os.Stat(path); err == nil {
			return path, nil
		}
	}

	return "", fmt.Errorf("binary not found in any location")
}

// Install 安装二进制文件
func (i *BinaryInstaller) Install(ctx context.Context) error {
	if i.config == nil || !i.config.Enabled {
		return fmt.Errorf("installation is not enabled")
	}

	// 设置默认值
	if i.config.InstallDir == "" {
		return fmt.Errorf("installDir is required")
	}
	if i.config.BinaryName == "" {
		return fmt.Errorf("binaryName is required")
	}

	// 检查是否已安装
	binaryPath := i.GetBinaryPath()
	if !i.config.AutoUpdate {
		if _, err := os.Stat(binaryPath); err == nil {
			i.logger.Debug("binary already installed, skipping", "path", binaryPath)
			return nil
		}
	}

	// 构建下载 URL
	downloadURL := i.buildDownloadURL()
	i.logger.Info("downloading binary", "url", downloadURL, "version", i.config.Version)

	// 创建临时目录
	tmpDir, err := os.MkdirTemp("", "binary-install-*")
	if err != nil {
		return fmt.Errorf("failed to create temp dir: %w", err)
	}
	defer os.RemoveAll(tmpDir)

	// 使用 go-getter 下载
	downloadDest := filepath.Join(tmpDir, "download")
	client := &getter.Client{
		Ctx:  ctx,
		Src:  downloadURL,
		Dst:  downloadDest,
		Mode: getter.ClientModeAny,
	}

	if err := client.Get(); err != nil {
		return fmt.Errorf("failed to download: %w", err)
	}

	// 验证校验和（如果配置了）
	if i.config.ChecksumURL != "" {
		if err := i.verifyChecksum(ctx, downloadDest); err != nil {
			return fmt.Errorf("checksum verification failed: %w", err)
		}
	}

	// 处理下载的文件
	var sourceFile string
	if i.config.ExtractArchive {
		// 解压缩
		extractDir := filepath.Join(tmpDir, "extracted")
		if err := i.extractArchive(downloadDest, extractDir); err != nil {
			return fmt.Errorf("failed to extract archive: %w", err)
		}
		sourceFile = extractDir
	} else {
		sourceFile = downloadDest
	}

	// 安装到目标目录
	if err := i.installToDir(sourceFile); err != nil {
		return fmt.Errorf("failed to install: %w", err)
	}

	i.logger.Info("binary installed successfully", "path", binaryPath)
	return nil
}

// buildDownloadURL 构建下载 URL（支持变量替换）
func (i *BinaryInstaller) buildDownloadURL() string {
	url := i.config.DownloadURL
	url = strings.ReplaceAll(url, "{{.Version}}", i.config.Version)
	url = strings.ReplaceAll(url, "{{.OS}}", runtime.GOOS)
	url = strings.ReplaceAll(url, "{{.Arch}}", runtime.GOARCH)
	return url
}

// verifyChecksum 验证校验和
func (i *BinaryInstaller) verifyChecksum(ctx context.Context, filePath string) error {
	// 下载校验和文件
	checksumPath := filePath + ".checksum"
	client := &getter.Client{
		Ctx:  ctx,
		Src:  i.config.ChecksumURL,
		Dst:  checksumPath,
		Mode: getter.ClientModeFile,
	}
	if err := client.Get(); err != nil {
		return err
	}
	defer os.Remove(checksumPath)

	// 读取预期校验和
	expectedData, err := os.ReadFile(checksumPath)
	if err != nil {
		return err
	}
	expectedChecksum := strings.TrimSpace(string(expectedData))

	// 计算实际校验和
	file, err := os.Open(filePath)
	if err != nil {
		return err
	}
	defer file.Close()

	var actualChecksum string
	checksumType := i.config.ChecksumType
	if checksumType == "" {
		checksumType = "sha256"
	}

	switch checksumType {
	case "sha256":
		hash := sha256.New()
		if _, err := io.Copy(hash, file); err != nil {
			return err
		}
		actualChecksum = hex.EncodeToString(hash.Sum(nil))
	default:
		return fmt.Errorf("unsupported checksum type: %s", checksumType)
	}

	// 比较（校验和文件可能包含文件名等其他信息）
	if !strings.Contains(expectedChecksum, actualChecksum) {
		return fmt.Errorf("checksum mismatch: expected %s, got %s", expectedChecksum, actualChecksum)
	}

	return nil
}

// extractArchive 解压归档文件
func (i *BinaryInstaller) extractArchive(archivePath, destDir string) error {
	// 只支持 tar.gz（最常见）
	file, err := os.Open(archivePath)
	if err != nil {
		return err
	}
	defer file.Close()

	gzr, err := gzip.NewReader(file)
	if err != nil {
		return err
	}
	defer gzr.Close()

	tr := tar.NewReader(gzr)
	stripComponents := i.config.StripComponents

	for {
		header, err := tr.Next()
		if err == io.EOF {
			break
		}
		if err != nil {
			return err
		}

		// 处理 StripComponents
		name := header.Name
		if stripComponents > 0 {
			parts := strings.Split(name, "/")
			if len(parts) <= stripComponents {
				continue
			}
			name = strings.Join(parts[stripComponents:], "/")
		}

		target := filepath.Join(destDir, name)

		// 安全检查：防止路径穿越
		if !strings.HasPrefix(target, filepath.Clean(destDir)+string(os.PathSeparator)) {
			continue
		}

		switch header.Typeflag {
		case tar.TypeDir:
			if err := os.MkdirAll(target, 0755); err != nil {
				return err
			}
		case tar.TypeReg:
			if err := os.MkdirAll(filepath.Dir(target), 0755); err != nil {
				return err
			}

			outFile, err := os.Create(target)
			if err != nil {
				return err
			}

			if _, err := io.Copy(outFile, tr); err != nil {
				outFile.Close()
				return err
			}
			outFile.Close()

			if err := os.Chmod(target, os.FileMode(header.Mode)); err != nil {
				return err
			}
		}
	}

	return nil
}

// installToDir 安装到目标目录
func (i *BinaryInstaller) installToDir(sourceDir string) error {
	// 删除旧的安装目录
	if err := os.RemoveAll(i.config.InstallDir); err != nil && !os.IsNotExist(err) {
		return err
	}

	// 创建父目录
	if err := os.MkdirAll(filepath.Dir(i.config.InstallDir), 0755); err != nil {
		return err
	}

	// 移动到安装目录
	return os.Rename(sourceDir, i.config.InstallDir)
}

// GetBinaryPath 获取安装的二进制路径
func (i *BinaryInstaller) GetBinaryPath() string {
	return filepath.Join(i.config.InstallDir, i.config.BinaryName)
}
