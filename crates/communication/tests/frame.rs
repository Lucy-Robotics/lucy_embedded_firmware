use lucy_embedded_firmware_communication::*;

const JOINTS: usize = 7;
const LEN: usize = frame_len(JOINTS);

fn command<const N: usize>(seq: u32) -> Frame<N> {
    let mut torque_enabled = [false; N];
    for (i, t) in torque_enabled.iter_mut().enumerate() {
        *t = i % 2 == 0;
    }
    Frame::new(seq, Body::Command(JointCommands {
        positions: [core::f64::consts::PI; N],
        velocities: [0.5; N],
        accelerations: [f64::from_bits(0xAAAA_AAAA_0000_0000); N],
        torque_enabled,
    }))
}

fn feed<const N: usize>(decoder: &mut FrameDecoder<'_, N>, bytes: &[u8]) -> Option<Result<Frame<N>, Error>> {
    let mut last = None;
    for &b in bytes {
        if let Some(r) = decoder.push_byte(b) {
            last = Some(r);
        }
    }
    last
}

#[test]
fn roundtrip() {
    let mut buf = [0u8; LEN];
    let n = encode(&command::<JOINTS>(42), &mut buf).unwrap();
    assert_eq!(buf[n - 1], 0);
    assert!(!buf[..n - 1].contains(&0));

    let mut rx = [0u8; LEN];
    assert_eq!(feed(&mut FrameDecoder::<JOINTS>::new(&mut rx), &buf[..n]), Some(Ok(command(42))));
}

#[test]
fn frame_len_fits_worst_case() {
    let mut buf = [0u8; LEN];
    assert!(encode(&command::<JOINTS>(u32::MAX), &mut buf).is_ok());
}

#[test]
fn supports_more_than_32_joints() {
    let mut buf = [0u8; frame_len(40)];
    let n = encode(&command::<40>(1), &mut buf).unwrap();

    let mut rx = [0u8; frame_len(40)];
    assert_eq!(feed(&mut FrameDecoder::<40>::new(&mut rx), &buf[..n]), Some(Ok(command(1))));
}

#[test]
fn resyncs_after_truncated_frame() {
    let mut buf = [0u8; LEN];
    let n = encode(&command::<JOINTS>(1), &mut buf).unwrap();
    let mut rx = [0u8; LEN];
    let mut decoder = FrameDecoder::<JOINTS>::new(&mut rx);
    assert!(matches!(feed(&mut decoder, &buf[n / 2..n]), Some(Err(_))));
    assert_eq!(feed(&mut decoder, &buf[..n]), Some(Ok(command(1))));
}

#[test]
fn detects_corruption() {
    let mut buf = [0u8; LEN];
    let n = encode(&command::<JOINTS>(1), &mut buf).unwrap();
    buf[n / 2] ^= 0x01;
    let mut rx = [0u8; LEN];
    assert!(matches!(feed(&mut FrameDecoder::<JOINTS>::new(&mut rx), &buf[..n]), Some(Err(_))));
}

#[test]
fn rejects_other_version() {
    let mut frame = command::<JOINTS>(1);
    frame.version = PROTOCOL_VERSION + 1;
    let mut buf = [0u8; LEN];
    let n = encode(&frame, &mut buf).unwrap();
    let mut rx = [0u8; LEN];
    assert_eq!(
        feed(&mut FrameDecoder::<JOINTS>::new(&mut rx), &buf[..n]),
        Some(Err(Error::Version(PROTOCOL_VERSION + 1)))
    );
}

#[test]
fn rejects_other_joint_count() {
    let mut buf = [0u8; frame_len(10)];
    let n = encode(&command::<10>(1), &mut buf).unwrap();
    let mut rx = [0u8; frame_len(10)];
    assert_eq!(
        feed(&mut FrameDecoder::<JOINTS>::new(&mut rx), &buf[..n]),
        Some(Err(Error::JointCount { expected: JOINTS as u32, received: 10 }))
    );
}

#[test]
fn reports_overflow() {
    let mut buf = [0u8; LEN];
    let n = encode(&command::<JOINTS>(1), &mut buf).unwrap();
    let mut rx = [0u8; 16];
    assert_eq!(
        feed(&mut FrameDecoder::<JOINTS>::new(&mut rx), &buf[..n]),
        Some(Err(Error::Overflow))
    );
}
