/*
Copyright 2025.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0
*/

package collector

import (
	"net"
	"os"
	"runtime"
	"time"

	"github.com/shirou/gopsutil/v3/cpu"
	"github.com/shirou/gopsutil/v3/disk"
	"github.com/shirou/gopsutil/v3/host"
	"github.com/shirou/gopsutil/v3/mem"
	netutil "github.com/shirou/gopsutil/v3/net"
	metav1 "k8s.io/apimachinery/pkg/apis/meta/v1"

	robotv1alpha1 "github.com/hxndghxndg/k8s4r/api/v1alpha1"
)

// CollectDeviceInfo 采集完整的设备信息（类似 psutil）
func CollectDeviceInfo() *robotv1alpha1.DeviceInfo {
	info := &robotv1alpha1.DeviceInfo{}

	// 获取主机名
	if hostname, err := os.Hostname(); err == nil {
		info.Hostname = hostname
	}

	// 采集平台信息
	info.Platform = collectPlatformInfo()

	// 采集 CPU 信息
	info.CPU = collectCPUInfo()

	// 采集内存信息
	info.Memory = collectMemoryInfo()

	// 采集交换分区信息
	info.Swap = collectSwapInfo()

	// 采集磁盘信息
	info.Disks = collectDiskInfo()

	// 采集网络接口信息
	info.NetworkInterfaces = collectNetworkInfo()

	// 采集系统启动时间和运行时间
	if hostInfo, err := host.Info(); err == nil {
		bootTime := metav1.NewTime(time.Unix(int64(hostInfo.BootTime), 0))
		info.BootTime = &bootTime
		info.Uptime = int64(hostInfo.Uptime)
	}

	// 设置更新时间
	now := metav1.Now()
	info.LastUpdated = &now

	return info
}

// collectPlatformInfo 采集平台信息
func collectPlatformInfo() robotv1alpha1.PlatformInfo {
	platformInfo := robotv1alpha1.PlatformInfo{
		OS:   runtime.GOOS,
		Arch: runtime.GOARCH,
	}

	if hostInfo, err := host.Info(); err == nil {
		platformInfo.Platform = hostInfo.Platform
		platformInfo.PlatformFamily = hostInfo.PlatformFamily
		platformInfo.PlatformVersion = hostInfo.PlatformVersion
		platformInfo.KernelVersion = hostInfo.KernelVersion
		platformInfo.KernelArch = hostInfo.KernelArch
	}

	return platformInfo
}

// collectCPUInfo 采集 CPU 信息
func collectCPUInfo() robotv1alpha1.CPUInfo {
	cpuInfo := robotv1alpha1.CPUInfo{}

	// 物理核心数
	if physicalCount, err := cpu.Counts(false); err == nil {
		cpuInfo.PhysicalCores = physicalCount
	}

	// 逻辑核心数
	if logicalCount, err := cpu.Counts(true); err == nil {
		cpuInfo.LogicalCores = logicalCount
	}

	// CPU 详细信息
	if infos, err := cpu.Info(); err == nil && len(infos) > 0 {
		cpuInfo.ModelName = infos[0].ModelName
		cpuInfo.Vendor = infos[0].VendorID
		cpuInfo.Mhz = infos[0].Mhz
	}

	// CPU 总使用率
	if percentages, err := cpu.Percent(time.Second, false); err == nil && len(percentages) > 0 {
		cpuInfo.UsagePercent = percentages[0]
	}

	// 每个 CPU 核心的使用率
	if perCPU, err := cpu.Percent(time.Second, true); err == nil {
		cpuInfo.PerCPUUsage = perCPU
	}

	return cpuInfo
}

// collectMemoryInfo 采集内存信息
func collectMemoryInfo() robotv1alpha1.MemoryInfo {
	memInfo := robotv1alpha1.MemoryInfo{}

	if vmStat, err := mem.VirtualMemory(); err == nil {
		memInfo.Total = vmStat.Total
		memInfo.Available = vmStat.Available
		memInfo.Used = vmStat.Used
		memInfo.Free = vmStat.Free
		memInfo.UsagePercent = vmStat.UsedPercent
		memInfo.Buffers = vmStat.Buffers
		memInfo.Cached = vmStat.Cached
	}

	return memInfo
}

// collectSwapInfo 采集交换分区信息
func collectSwapInfo() robotv1alpha1.SwapInfo {
	swapInfo := robotv1alpha1.SwapInfo{}

	if swapStat, err := mem.SwapMemory(); err == nil {
		swapInfo.Total = swapStat.Total
		swapInfo.Used = swapStat.Used
		swapInfo.Free = swapStat.Free
		swapInfo.UsagePercent = swapStat.UsedPercent
	}

	return swapInfo
}

// collectDiskInfo 采集磁盘分区信息
func collectDiskInfo() []robotv1alpha1.DiskInfo {
	var disks []robotv1alpha1.DiskInfo

	partitions, err := disk.Partitions(false)
	if err != nil {
		return disks
	}

	for _, partition := range partitions {
		usage, err := disk.Usage(partition.Mountpoint)
		if err != nil {
			continue
		}

		diskInfo := robotv1alpha1.DiskInfo{
			Device:       partition.Device,
			MountPoint:   partition.Mountpoint,
			FSType:       partition.Fstype,
			Total:        usage.Total,
			Used:         usage.Used,
			Free:         usage.Free,
			UsagePercent: usage.UsedPercent,
			InodesTotal:  usage.InodesTotal,
			InodesUsed:   usage.InodesUsed,
			InodesFree:   usage.InodesFree,
		}

		disks = append(disks, diskInfo)
	}

	return disks
}

// collectNetworkInfo 采集网络接口信息
func collectNetworkInfo() []robotv1alpha1.NetworkInterface {
	var interfaces []robotv1alpha1.NetworkInterface

	// 获取网卡列表
	netInterfaces, err := net.Interfaces()
	if err != nil {
		return interfaces
	}

	// 获取网络 IO 统计
	ioCounters, _ := netutil.IOCounters(true)
	ioMap := make(map[string]netutil.IOCountersStat)
	for _, io := range ioCounters {
		ioMap[io.Name] = io
	}

	for _, iface := range netInterfaces {
		// 跳过回环接口
		if iface.Flags&net.FlagLoopback != 0 {
			continue
		}

		netInterface := robotv1alpha1.NetworkInterface{
			Name:         iface.Name,
			HardwareAddr: iface.HardwareAddr.String(),
			MTU:          iface.MTU,
		}

		// 获取 IP 地址（包含子网掩码）
		addrs, err := iface.Addrs()
		if err == nil {
			for _, addr := range addrs {
				netInterface.Addresses = append(netInterface.Addresses, addr.String())
			}
		}

		// 解析标志位
		var flags []string
		if iface.Flags&net.FlagUp != 0 {
			flags = append(flags, "UP")
		}
		if iface.Flags&net.FlagBroadcast != 0 {
			flags = append(flags, "BROADCAST")
		}
		if iface.Flags&net.FlagMulticast != 0 {
			flags = append(flags, "MULTICAST")
		}
		if iface.Flags&net.FlagPointToPoint != 0 {
			flags = append(flags, "POINTOPOINT")
		}
		netInterface.Flags = flags

		// 添加网络 IO 统计
		if io, ok := ioMap[iface.Name]; ok {
			netInterface.BytesSent = io.BytesSent
			netInterface.BytesRecv = io.BytesRecv
			netInterface.PacketsSent = io.PacketsSent
			netInterface.PacketsRecv = io.PacketsRecv
			netInterface.Errin = io.Errin
			netInterface.Errout = io.Errout
			netInterface.Dropin = io.Dropin
			netInterface.Dropout = io.Dropout
		}

		interfaces = append(interfaces, netInterface)
	}

	return interfaces
}
