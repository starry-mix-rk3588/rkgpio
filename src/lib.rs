//! RK3588 GPIO driver (Rust version)
//!
#![no_std]
#![feature(const_nonnull_new)]

use core::ptr::NonNull;
use core::ptr::{read_volatile, write_volatile};
use log::*;
use tock_registers::{
    interfaces::{Readable, Writeable},
    register_structs,
    registers::ReadWrite,
};

// CRU（Clock & Reset Unit）基地址（由 DTS/axconfig 确定，默认根据仓库配置）
const CRU_BASE: usize = 0xFD7C0000;

// CRU CLKGATE 偏移：这里保留用于示例/手动使能，具体值以 board dts/axconfig 为准
const CRU_CLKGATE_CON32_OFFSET: usize = 0x0320;
const CRU_CLKGATE_CON35_OFFSET: usize = 0x0350;

// GPIO 类型标识（与 Linux driver 保持一致）
const GPIO_TYPE_V1: u32 = 0;
const GPIO_TYPE_V2: u32 = 0x01000C2B;
const GPIO_TYPE_V2_1: u32 = 0x0101157C;

// RK3588 GPIO v2 偏移（参考 Linux 定义）
const OFF_DATA: usize = 0x00; // port_dr
const OFF_DIR: usize = 0x08; // port_ddr
const OFF_INT_EN: usize = 0x10; // int_en
const OFF_INT_MASK: usize = 0x18; // int_mask
const OFF_INT_TYPE: usize = 0x20; // int_type
const OFF_INT_POL: usize = 0x28; // int_polarity
const OFF_INT_BOTHEDGE: usize = 0x30; // int_bothedge
const OFF_DEBOUNCE: usize = 0x38; // debounce
const OFF_DBCLK_DIV_EN: usize = 0x40; // dbclk_div_en
const OFF_DBCLK_DIV_CON: usize = 0x48; // dbclk_div_con
const OFF_INT_STATUS: usize = 0x50; // int_status
const OFF_INT_RAWSTATUS: usize = 0x58; // int_rawstatus
const OFF_PORT_EOI: usize = 0x60; // port_eoi
const OFF_EXT_PORT: usize = 0x70; // ext_port
const OFF_VERSION_ID: usize = 0x78; // version id

/// 设置 CRU 门控寄存器（简单实现：写入带写使能的两次写法）
fn cru_set_gate(offset: usize, bit: u32) {
    let reg = (CRU_BASE + offset) as *mut u32;
    unsafe { write_volatile(reg, (1 << bit) | (1 << (bit + 16))) }
}

/// v2 需要特殊的 32-bit 两次写法以支持按位写（Linux 的 gpio_writel_v2）
unsafe fn gpio_writel_v2(reg: *mut u32, val: u32) {
    // 低 16 位写到 reg，同时写入 0xffff0000 作为写使能
    write_volatile(reg, (val & 0xffff) | 0xffff0000);
    // 高 16 位写到 reg+0x4
    write_volatile(reg.add(1), (val >> 16) | 0xffff0000);
}

unsafe fn gpio_readl_v2(reg: *mut u32) -> u32 {
    let low = read_volatile(reg);
    let high = read_volatile(reg.add(1));
    (high << 16) | (low & 0xffff)
}

// IOMUX 寄存器定义（保留现有实现）
register_structs! {
    pub IomuxRegisters {
        (0x000 => gpio0a_iomux: [ReadWrite<u32>; 4]),
        (0x010 => gpio0b_iomux: [ReadWrite<u32>; 4]),
        (0x020 => gpio0c_iomux: [ReadWrite<u32>; 4]),
        (0x030 => gpio0d_iomux: [ReadWrite<u32>; 4]),
        (0x040 => gpio1a_iomux: [ReadWrite<u32>; 4]),
        (0x050 => gpio1b_iomux: [ReadWrite<u32>; 4]),
        (0x060 => gpio1c_iomux: [ReadWrite<u32>; 4]),
        (0x070 => gpio1d_iomux: [ReadWrite<u32>; 4]),
        (0x080 => gpio2a_iomux: [ReadWrite<u32>; 4]),
        (0x090 => gpio2b_iomux: [ReadWrite<u32>; 4]),
        (0x0a0 => gpio2c_iomux: [ReadWrite<u32>; 4]),
        (0x0b0 => gpio2d_iomux: [ReadWrite<u32>; 4]),
        (0x0c0 => gpio3a_iomux: [ReadWrite<u32>; 4]),
        (0x0d0 => gpio3b_iomux: [ReadWrite<u32>; 4]),
        (0x0e0 => gpio3c_iomux: [ReadWrite<u32>; 4]),
        (0x0f0 => gpio3d_iomux: [ReadWrite<u32>; 4]),
        (0x100 => gpio4a_iomux: [ReadWrite<u32>; 4]),
        (0x110 => gpio4b_iomux: [ReadWrite<u32>; 4]),
        (0x120 => gpio4c_iomux: [ReadWrite<u32>; 4]),
        (0x130 => gpio4d_iomux: [ReadWrite<u32>; 4]),
        (0x140 => @END),
    }
}

unsafe impl Send for Gpio {}
unsafe impl Sync for Gpio {}

unsafe impl Send for Iomux {}
unsafe impl Sync for Iomux {}

pub struct Gpio {
    base: NonNull<u8>,
    gpio_type: u32,
}

pub struct Iomux {
    registers: NonNull<IomuxRegisters>,
}

impl Gpio {
    /// 创建新的 GPIO 实例（只做指针包装，实际检测在 gpio_init）
    pub const fn new(registers: *mut u8) -> Self {
        Self {
            base: NonNull::new(registers).unwrap(),
            gpio_type: GPIO_TYPE_V1,
        }
    }

    /// 读取寄存器（根据 gpio_type 自动选择 v1/v2 语义）
    fn read_reg(&self, offset: usize) -> u32 {
        let reg = unsafe { self.base.as_ptr().add(offset) } as *mut u32;
        if self.gpio_type == GPIO_TYPE_V2 {
            unsafe { gpio_readl_v2(reg) }
        } else {
            unsafe { read_volatile(reg) }
        }
    }

    /// 写寄存器（根据 gpio_type 自动选择 v1/v2 语义）
    fn write_reg(&mut self, offset: usize, val: u32) {
        let reg = unsafe { self.base.as_ptr().add(offset) } as *mut u32;
        if self.gpio_type == GPIO_TYPE_V2 {
            unsafe { gpio_writel_v2(reg, val) }
        } else {
            unsafe { write_volatile(reg, val) }
        }
    }

    /// 按位写（支持 v2 的按位写优化）
    fn write_bit(&mut self, bit: u32, value: bool, offset: usize) {
        let reg = unsafe { self.base.as_ptr().add(offset) } as *mut u32;
        if self.gpio_type == GPIO_TYPE_V2 {
            // v2: write to reg or reg+4 depending on bit >=16
            let idx = if bit >= 16 { 1 } else { 0 };
            let b = (bit % 16) as u32;
            let data = if value {
                (1u32 << b) | (1u32 << (b + 16))
            } else {
                1u32 << (b + 16)
            };
            unsafe { write_volatile(reg.add(idx), data) }
        } else {
            // v1: read-modify-write
            let mut cur = unsafe { read_volatile(reg) };
            if value {
                cur |= 1u32 << bit;
            } else {
                cur &= !(1u32 << bit);
            }
            unsafe { write_volatile(reg, cur) }
        }
    }

    /// 按位读
    fn read_bit(&self, bit: u32, offset: usize) -> bool {
        let reg = unsafe { self.base.as_ptr().add(offset) } as *mut u32;
        if self.gpio_type == GPIO_TYPE_V2 {
            let idx = if bit >= 16 { 1 } else { 0 };
            let data = unsafe { read_volatile(reg.add(idx)) };
            ((data >> (bit % 16)) & 1) != 0
        } else {
            let data = unsafe { read_volatile(reg) };
            ((data >> bit) & 1) != 0
        }
    }

    /// 初始化：打开时钟并检测 GPIO 版本（v1/v2），同时做默认屏蔽/清理
    pub fn gpio_init(&mut self) {
        // 先打开 GPIO3 时钟（示例），实际 bit/offset 请根据 board DTS 修改
        cru_set_gate(CRU_CLKGATE_CON35_OFFSET, 0);

        // 读取 version id（仅对 v2 可用），若读不到则默认 v1
        let ver_addr = unsafe { self.base.as_ptr().add(OFF_VERSION_ID) } as *mut u32;
        let version = unsafe { read_volatile(ver_addr) };
        if version == GPIO_TYPE_V2 || version == GPIO_TYPE_V2_1 {
            self.gpio_type = GPIO_TYPE_V2;
            info!("GPIO: v2 detected");
        } else {
            self.gpio_type = GPIO_TYPE_V1;
            info!("GPIO: v1 detected");
        }

        // 禁用所有中断并清除状态
        self.write_reg(OFF_INT_EN, 0);
        self.write_reg(OFF_INT_MASK, 0xffffffff);
        self.write_reg(OFF_PORT_EOI, 0xffffffff);

        // 默认设置所有 GPIO 为输入
        self.write_reg(OFF_DIR, 1);
    }

    /// 设置 GPIO 输出电平（全局编号）
    pub fn gpio_write(&mut self, pin: u8, val: u8) {
        let bit = (pin % 32) as u32;
        self.write_bit(bit, val != 0, OFF_DATA);
    }

    /// 读取 GPIO 电平（全局编号）
    pub fn gpio_read(&self, pin: u8) -> u8 {
        let bit = (pin % 32) as u32;
        if self.read_bit(bit, OFF_DATA) {
            1
        } else {
            0
        }
    }

    /// 设置方向，is_output=1 为输出
    pub fn gpio_setdir(&mut self, pin: u8, is_output: u8) {
        let bit = (pin % 32) as u32;
        self.write_bit(bit, is_output != 0, OFF_DIR);
    }

    /// 读取方向
    pub fn gpio_getdir(&self, pin: u8) -> u8 {
        let bit = (pin % 32) as u32;
        if self.read_bit(bit, OFF_DIR) {
            1
        } else {
            0
        }
    }

    /// 设置中断类型（level/edge）和极性（high/low）
    pub fn set_interrupt_type(&mut self, pin: u8, is_level: bool, is_high: bool) {
        let bit = (pin % 32) as u32;
        // int_type: 1=edge, 0=level (Linux driver uses opposite for v2 compatibility)
        let mut cur = self.read_reg(OFF_INT_TYPE);
        if is_level {
            cur &= !(1 << bit);
        } else {
            cur |= 1 << bit;
        }
        self.write_reg(OFF_INT_TYPE, cur);

        let mut pol = self.read_reg(OFF_INT_POL);
        if is_high {
            pol |= 1 << bit;
        } else {
            pol &= !(1 << bit);
        }
        self.write_reg(OFF_INT_POL, pol);
    }

    /// 使能或禁止中断
    pub fn enable_interrupt(&mut self, pin: u8, enabled: bool) {
        let bit = (pin % 32) as u32;
        let mut en = self.read_reg(OFF_INT_EN);
        let mut mask = self.read_reg(OFF_INT_MASK);
        if enabled {
            en |= 1 << bit;
            mask &= !(1 << bit);
        } else {
            en &= !(1 << bit);
            mask |= 1 << bit;
        }
        self.write_reg(OFF_INT_EN, en);
        self.write_reg(OFF_INT_MASK, mask);
    }
}

impl Iomux {
    /// 创建新的IOMUX实例
    pub const fn new(registers: *mut u8) -> Self {
        Self {
            registers: NonNull::new(registers).unwrap().cast(),
        }
    }

    const fn iomux_regs(&self) -> &IomuxRegisters {
        unsafe { self.registers.as_ref() }
    }

    /// 设置 GPIO 复用功能
    pub fn set_iomux(&mut self, gpio_num: u16, function: u32) {
        let bank = gpio_num / 32; // GPIO 组号 (0-4)
        let pin = gpio_num % 32; // 组内引脚编号 (0-31)
        let sub_bank = pin / 8; // 每组内的子寄存器索引 (0-3)
        let sub_pin = pin % 8; // 子寄存器内的引脚偏移 (0-7)

        let reg_array = match bank {
            0 => match pin / 8 {
                0 => &self.iomux_regs().gpio0a_iomux,
                1 => &self.iomux_regs().gpio0b_iomux,
                2 => &self.iomux_regs().gpio0c_iomux,
                3 => &self.iomux_regs().gpio0d_iomux,
                _ => unreachable!(),
            },
            1 => match pin / 8 {
                0 => &self.iomux_regs().gpio1a_iomux,
                1 => &self.iomux_regs().gpio1b_iomux,
                2 => &self.iomux_regs().gpio1c_iomux,
                3 => &self.iomux_regs().gpio1d_iomux,
                _ => unreachable!(),
            },
            2 => match pin / 8 {
                0 => &self.iomux_regs().gpio2a_iomux,
                1 => &self.iomux_regs().gpio2b_iomux,
                2 => &self.iomux_regs().gpio2c_iomux,
                3 => &self.iomux_regs().gpio2d_iomux,
                _ => unreachable!(),
            },
            3 => match pin / 8 {
                0 => &self.iomux_regs().gpio3a_iomux,
                1 => &self.iomux_regs().gpio3b_iomux,
                2 => &self.iomux_regs().gpio3c_iomux,
                3 => &self.iomux_regs().gpio3d_iomux,
                _ => unreachable!(),
            },
            4 => match pin / 8 {
                0 => &self.iomux_regs().gpio4a_iomux,
                1 => &self.iomux_regs().gpio4b_iomux,
                2 => &self.iomux_regs().gpio4c_iomux,
                3 => &self.iomux_regs().gpio4d_iomux,
                _ => unreachable!(),
            },
            _ => return,
        };

        // 每个引脚占用4位
        let shift = (sub_pin * 4) as usize;
        let mask = 0xF << shift;
        let idx = sub_bank as usize;
        let val = reg_array[idx].get();
        reg_array[idx].set((val & !mask) | ((function & 0xF) << shift));
    }

    /// GPIO 复用配置初始化
    pub fn iomux_init(&mut self) {
        // 这里添加默认的 GPIO 复用配置
        // RK3588 的 GPIO 功能通常是 function=0
        self.set_iomux(0, 0); // 配置 GPIO0_A0 为 GPIO 功能
    }
}
