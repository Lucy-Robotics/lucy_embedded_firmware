use lucy_embedded_firmware_rp2040_support::picotool_reset::bootsel_activity_led_mask;


    #[test]
    fn bootsel_mask_unset_when_bit8_clear() {
        assert_eq!(bootsel_activity_led_mask(0), 0);
        assert_eq!(bootsel_activity_led_mask(0x7f), 0);
    }

    #[test]
    fn bootsel_mask_clamps_pin_to_five_bits() {
        // bit8 set, pin field = 25 → GPIO25
        assert_eq!(bootsel_activity_led_mask(0x100 | (25 << 9)), 1u32 << 25);
        // Hostile: huge shift field still masks to pin 31
        assert_eq!(bootsel_activity_led_mask(0x100 | (0xffff << 9)), 1u32 << 31);
    }
