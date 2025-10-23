pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

pub struct Twist {
    pub linear: Vector3,
    pub angular: Vector3,
}

pub(crate) fn serialize_twist(twist: &Twist, buf: &'_ mut [u8]) -> usize {
    let mut offset = 0;

    fn write_f64_le(value: f64, buf: &mut [u8], offset: &mut usize) {
        let bytes = value.to_le_bytes();
        buf[*offset..*offset + 8].copy_from_slice(&bytes);
        *offset += 8;
    }

    buf[0..4].copy_from_slice(&[0x00, 0x01, 0x00, 0x00]);
    offset += 4;

    write_f64_le(twist.linear.x, buf, &mut offset);
    write_f64_le(twist.linear.y, buf, &mut offset);
    write_f64_le(twist.linear.z, buf, &mut offset);
    write_f64_le(twist.angular.x, buf, &mut offset);
    write_f64_le(twist.angular.y, buf, &mut offset);
    write_f64_le(twist.angular.z, buf, &mut offset);

    offset
}
