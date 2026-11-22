use lucy_embedded_firmware_core::utils::map_range;

#[test]
fn map_range_extrapolates_instead_of_clamping() {
    assert_eq!(map_range(-1.0, 0.0, 1.0, 0.0, 100.0), -100.0);
    assert_eq!(map_range(2.0, 0.0, 1.0, 0.0, 100.0), 200.0);
    assert_eq!(map_range(2.0, 0.0, 1.0, 100.0, 0.0), -100.0);
}

#[test]
fn map_range_degenerate_input_span_returns_out_min() {
    assert_eq!(map_range(5.0, 1.0, 1.0, 10.0, 20.0), 10.0);
}
