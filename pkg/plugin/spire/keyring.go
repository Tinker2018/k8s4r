package spire

import (
	"fmt"
	"unsafe"

	"github.com/hashicorp/go-hclog"
	"golang.org/x/sys/unix"
)

// KeyringManager 管理 Linux Keyring 中的密钥
type KeyringManager struct {
	logger    hclog.Logger
	config    *KeyringConfig
	keyringID int // Keyring ID
}

// Keyring 类型常量
const (
	KeyringTypeSession = -3 // KEY_SPEC_SESSION_KEYRING
	KeyringTypeUser    = -4 // KEY_SPEC_USER_KEYRING
	KeyringTypeProcess = -2 // KEY_SPEC_PROCESS_KEYRING
)

// NewKeyringManager 创建 KeyringManager
func NewKeyringManager(logger hclog.Logger, config *KeyringConfig) (*KeyringManager, error) {
	if !config.Enabled {
		return nil, fmt.Errorf("keyring not enabled")
	}

	km := &KeyringManager{
		logger: logger,
		config: config,
	}

	// 初始化 keyring
	if err := km.initialize(); err != nil {
		return nil, err
	}

	return km, nil
}

// initialize 初始化 keyring
func (km *KeyringManager) initialize() error {
	// 根据配置选择 keyring 类型
	var keyringType int
	switch km.config.Type {
	case "session":
		keyringType = KeyringTypeSession
	case "user":
		keyringType = KeyringTypeUser
	case "process":
		keyringType = KeyringTypeProcess
	default:
		return fmt.Errorf("invalid keyring type: %s", km.config.Type)
	}

	km.keyringID = keyringType
	km.logger.Info("initialized keyring", "type", km.config.Type, "id", km.keyringID)

	return nil
}

// StoreKey 存储密钥到 keyring
func (km *KeyringManager) StoreKey(name string, data []byte) (int, error) {
	keyID, err := unix.AddKey(
		"user",       // key type
		name,         // key description
		data,         // key payload
		km.keyringID, // destination keyring
	)

	if err != nil {
		return 0, fmt.Errorf("failed to add key to keyring: %w", err)
	}

	km.logger.Debug("key stored in keyring", "name", name, "keyID", keyID)
	return keyID, nil
}

// ReadKey 从 keyring 读取密钥
func (km *KeyringManager) ReadKey(keyID int) ([]byte, error) {
	// 首先获取密钥大小
	size, err := unix.KeyctlInt(unix.KEYCTL_READ, keyID, 0, 0, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to get key size: %w", err)
	}

	// 读取密钥数据
	buffer := make([]byte, size)
	n, err := unix.KeyctlBuffer(unix.KEYCTL_READ, keyID, buffer, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to read key: %w", err)
	}

	km.logger.Debug("key read from keyring", "keyID", keyID, "size", n)
	return buffer[:n], nil
}

// SearchKey 在 keyring 中搜索密钥
func (km *KeyringManager) SearchKey(name string) (int, error) {
	keyID, err := unix.KeyctlSearch(
		km.keyringID, // keyring to search
		"user",       // key type
		name,         // key description
		0,            // destination keyring (0 = don't link)
	)

	if err != nil {
		return 0, fmt.Errorf("key not found in keyring: %w", err)
	}

	km.logger.Debug("key found in keyring", "name", name, "keyID", keyID)
	return keyID, nil
}

// RevokeKey 撤销密钥
func (km *KeyringManager) RevokeKey(keyID int) error {
	_, err := unix.KeyctlInt(unix.KEYCTL_REVOKE, keyID, 0, 0, 0)
	if err != nil {
		return fmt.Errorf("failed to revoke key: %w", err)
	}

	km.logger.Debug("key revoked", "keyID", keyID)
	return nil
}

// SetPermissions 设置密钥权限
// perm: 0x3f0f0000 = KEY_POS_ALL | KEY_USR_ALL (owner 和 user 都有完全权限)
func (km *KeyringManager) SetPermissions(keyID int, perm uint32) error {
	_, err := unix.KeyctlInt(unix.KEYCTL_SETPERM, keyID, int(perm), 0, 0)
	if err != nil {
		return fmt.Errorf("failed to set permissions: %w", err)
	}

	km.logger.Debug("key permissions set", "keyID", keyID, "perm", fmt.Sprintf("0x%x", perm))
	return nil
}

// LinkKey 将密钥链接到目标 keyring
func (km *KeyringManager) LinkKey(keyID, targetKeyringID int) error {
	_, err := unix.KeyctlInt(unix.KEYCTL_LINK, keyID, targetKeyringID, 0, 0)
	if err != nil {
		return fmt.Errorf("failed to link key: %w", err)
	}

	km.logger.Debug("key linked", "keyID", keyID, "targetKeyring", targetKeyringID)
	return nil
}

// ListKeys 列出 keyring 中的所有密钥
func (km *KeyringManager) ListKeys() ([]int, error) {
	// 获取密钥列表大小
	size, err := unix.KeyctlInt(unix.KEYCTL_READ, km.keyringID, 0, 0, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to get keyring size: %w", err)
	}

	// 读取密钥列表
	buffer := make([]byte, size)
	n, err := unix.KeyctlBuffer(unix.KEYCTL_READ, km.keyringID, buffer, 0)
	if err != nil {
		return nil, fmt.Errorf("failed to read keyring: %w", err)
	}

	// 解析密钥 ID 列表（每个 ID 是 4 字节整数）
	keyCount := n / 4
	keyIDs := make([]int, keyCount)
	for i := 0; i < keyCount; i++ {
		keyID := *(*int32)(unsafe.Pointer(&buffer[i*4]))
		keyIDs[i] = int(keyID)
	}

	km.logger.Debug("listed keys in keyring", "count", keyCount)
	return keyIDs, nil
}

// Clear 清空 keyring 中的所有密钥
func (km *KeyringManager) Clear() error {
	_, err := unix.KeyctlInt(unix.KEYCTL_CLEAR, km.keyringID, 0, 0, 0)
	if err != nil {
		return fmt.Errorf("failed to clear keyring: %w", err)
	}

	km.logger.Info("keyring cleared")
	return nil
}

// GetKeyringID 获取当前使用的 keyring ID
func (km *KeyringManager) GetKeyringID() int {
	return km.keyringID
}
