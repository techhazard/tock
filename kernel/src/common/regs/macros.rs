#[macro_export]
macro_rules! register_single_bits {
    {
        $valtype:ty, $reg_desc:ident, $field:ident, $bit:expr, 1
    } => {
        #[allow(non_upper_case_globals)]
        #[allow(unused)]
        pub const SET: FieldValue<$valtype, $reg_desc> =
            FieldValue::<$valtype, $reg_desc>::new(1, $bit, 1);

        #[allow(non_upper_case_globals)]
        #[allow(unused)]
        pub const CLEAR: FieldValue<$valtype, $reg_desc> =
            FieldValue::<$valtype, $reg_desc>::new(1, $bit, 0);
    };
    {
        $valtype:ty, $reg_desc:ident, $field:ident, $bit:expr, $numbits:expr
    } => { };
}


#[macro_export]
macro_rules! register_bitmasks {
    {
        // BITFIELD_NAME OFFSET(x)
        $valtype:ty, $reg_desc:ident, [
            $( $field:ident OFFSET($offset:expr)),+
        ]
    } => {
        $( register_bitmasks!($valtype, $reg_desc, $field, $offset, 1, []); )*
    };
    {
        // BITFIELD_NAME OFFSET
        // All fields are 1 bit
        $valtype:ty, $reg_desc:ident, [
            $( $field:ident $offset:expr ),+
        ]
    } => {
        $( register_bitmasks!($valtype, $reg_desc, $field, $offset, 1, []); )*
    };

    {
        // BITFIELD_NAME OFFSET(x) NUMBITS(y)
        $valtype:ty, $reg_desc:ident, [
            $( $field:ident OFFSET($offset:expr) NUMBITS($numbits:expr) ),+
        ]
    } => {
        $( register_bitmasks!($valtype, $reg_desc, $field, $offset, $numbits, []); )*
    };

    {
        // BITFIELD_NAME OFFSET(x) NUMBITS(y) []
        $valtype:ty, $reg_desc:ident, [
            $( $field:ident OFFSET($offset:expr) NUMBITS($numbits:expr) $values:tt ),+
        ]
    } => {
        $( register_bitmasks!($valtype, $reg_desc, $field, $offset, $numbits, $values); )*
    };
    {
        $valtype:ty, $reg_desc:ident, $field:ident,
                    $offset:expr, $numbits:expr,
                    [$( $valname:ident = $value:expr ),*]
    } => {
        #[allow(non_upper_case_globals)]
        #[allow(unused)]
        pub const $field: Field<$valtype, $reg_desc> =
            Field::<$valtype, $reg_desc>::new((1<<$numbits)-1, $offset);

        #[allow(non_snake_case)]
        #[allow(unused)]
        pub mod $field {
            #[allow(unused_imports)]
            use $crate::common::regs::FieldValue;
            use super::$reg_desc;

            $(
            #[allow(non_upper_case_globals)]
            #[allow(unused)]
            pub const $valname: FieldValue<$valtype, $reg_desc> =
                FieldValue::<$valtype, $reg_desc>::new((1<<$numbits)-1, $offset, $value);
            )*

            register_single_bits!($valtype, $reg_desc, $field, $offset, $numbits);

            #[allow(dead_code)]
            #[allow(non_camel_case_types)]
            pub enum Value {
                $(
                    $valname = $value,
                )*
            }
        }
    };
}

#[macro_export]
macro_rules! register_bitfields {
    {
        $valtype:ty, $( $reg:ident $fields:tt ),*
    } => {
        $(
            #[allow(non_snake_case)]
            pub mod $reg {
                pub struct Register;
                impl $crate::common::regs::RegisterLongName for Register {}

                use $crate::common::regs::Field;

                register_bitmasks!( $valtype, Register, $fields );
            }
        )*
    }
}
