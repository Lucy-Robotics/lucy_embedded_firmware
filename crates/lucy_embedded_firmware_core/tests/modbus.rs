use crc::{CRC_16_MODBUS, Crc};
use lucy_embedded_firmware_core::modbus::{
    check_crc, inter_frame_delay_us, parse_modbus_frame, route_modbus_request, RegisterTable,
    RegisterView, Slave,
};

const MODBUS_CRC: Crc<u16> = Crc::<u16>::new(&CRC_16_MODBUS);

fn frame_write_single(slave: u8, addr: u16, value: u16) -> [u8; 8] {
    let mut buf = [0u8; 8];
    buf[0] = slave;
    buf[1] = 0x06;
    let a = addr.to_be_bytes();
    let v = value.to_be_bytes();
    buf[2] = a[0];
    buf[3] = a[1];
    buf[4] = v[0];
    buf[5] = v[1];
    let crc = MODBUS_CRC.checksum(&buf[..6]);
    let c = crc.to_le_bytes();
    buf[6] = c[0];
    buf[7] = c[1];
    buf
}

#[test]
fn inter_frame_delay_high_baud_uses_floor() {
    assert_eq!(inter_frame_delay_us(115_200), 1750);
    assert_eq!(inter_frame_delay_us(38_400), 1750);
}

#[test]
fn inter_frame_delay_low_baud_is_3_5_chars() {
    assert_eq!(inter_frame_delay_us(9600), 4011);
}

#[test]
fn write_single_echoes_and_stores() {
    let table = RegisterTable::new();
    let req_frame = frame_write_single(1, 2, 0x1234);
    let slave = Slave { address: 1 };
    let request = parse_modbus_frame(&slave, &req_frame).unwrap();
    let mut out = [0u8; 16];
    let n = route_modbus_request(1, &table, request, &mut out).unwrap();
    assert_eq!(n, 8);
    assert_eq!(&out[..8], &req_frame);
    assert_eq!(table.registers[2].get(), 0x1234);
}

#[test]
fn read_holding_registers_returns_values() {
    let table = RegisterTable::new();
    table.registers[0].set(0x0001);
    table.registers[1].set(0xABCD);

    let mut req = [0u8; 8];
    req[0] = 1;
    req[1] = 0x03;
    req[2] = 0x00;
    req[3] = 0x00;
    req[4] = 0x00;
    req[5] = 0x02;
    let crc = MODBUS_CRC.checksum(&req[..6]);
    let c = crc.to_le_bytes();
    req[6] = c[0];
    req[7] = c[1];

    let slave = Slave { address: 1 };
    let request = parse_modbus_frame(&slave, &req).unwrap();
    let mut out = [0u8; 32];
    let n = route_modbus_request(1, &table, request, &mut out).unwrap();
    assert_eq!(n, 9);
    assert_eq!(out[0], 1);
    assert_eq!(out[1], 0x03);
    assert_eq!(out[2], 4);
    assert_eq!(out[3], 0x00);
    assert_eq!(out[4], 0x01);
    assert_eq!(out[5], 0xAB);
    assert_eq!(out[6], 0xCD);
    assert!(check_crc(&out[..n]));
}

#[test]
fn register_view_rejects_index_equal_to_nb() {
    let table = RegisterTable::new();
    table.registers[0].set(42);
    let view = RegisterView::new(&table, 0, 1);
    assert_eq!(view.read_register(0), 42);
    assert_eq!(view.read_register(1), 0);
}
