/// Implements the 6LoWPAN specification for sending IPv6 datagrams over
/// 802.15.4 packets efficiently, as detailed in RFC 6282.
use core::mem;
use core::result::Result;
use net::ieee802154::MacAddress;
use net::ip::{IP6Header, IPAddr, ip6_nh};
use net::util;
use net::util::{slice_to_u16, u16_to_slice};

/// Contains bit masks and constants related to the two-byte header of the
/// LoWPAN_IPHC encoding format.
mod iphc {
    pub const DISPATCH: [u8; 2] = [0x60, 0x00];

    // First byte masks

    pub const TF_TRAFFIC_CLASS: u8 = 0x08;
    pub const TF_FLOW_LABEL: u8 = 0x10;

    pub const NH: u8 = 0x04;

    pub const HLIM_MASK: u8 = 0x03;
    pub const HLIM_INLINE: u8 = 0x00;
    pub const HLIM_1: u8 = 0x01;
    pub const HLIM_64: u8 = 0x02;
    pub const HLIM_255: u8 = 0x03;

    // Second byte masks

    pub const CID: u8 = 0x80;

    pub const SAC: u8 = 0x40;

    pub const SAM_MASK: u8 = 0x30;
    pub const SAM_INLINE: u8 = 0x00;
    pub const SAM_MODE1: u8 = 0x10;
    pub const SAM_MODE2: u8 = 0x20;
    pub const SAM_MODE3: u8 = 0x30;

    pub const MULTICAST: u8 = 0x08;

    pub const DAC: u8 = 0x04;
    pub const DAM_MASK: u8 = 0x03;
    pub const DAM_INLINE: u8 = 0x00;
    pub const DAM_MODE1: u8 = 0x01;
    pub const DAM_MODE2: u8 = 0x02;
    pub const DAM_MODE3: u8 = 0x03;

    // Address compression
    pub const MAC_BASE: [u8; 8] = [0, 0, 0, 0xff, 0xfe, 0, 0, 0];
    pub const MAC_UL: u8 = 0x02;
}

/// Contains bit masks and constants related to LoWPAN_NHC encoding,
/// including some specific to UDP header encoding
mod nhc {
    pub const DISPATCH_NHC: u8 = 0xe0;
    pub const DISPATCH_UDP: u8 = 0xf0;
    pub const DISPATCH_MASK: u8 = 0xf0;

    pub const EID_MASK: u8 = 0x0e;
    pub const HOP_OPTS: u8 = 0 << 1;
    pub const ROUTING: u8 = 1 << 1;
    pub const FRAGMENT: u8 = 2 << 1;
    pub const DST_OPTS: u8 = 3 << 1;
    pub const MOBILITY: u8 = 4 << 1;
    pub const IP6: u8 = 7 << 1;

    pub const NH: u8 = 0x01;

    // UDP header compression

    pub const UDP_4BIT_PORT: u16 = 0xf0b0;
    pub const UDP_4BIT_PORT_MASK: u16 = 0xfff0;
    pub const UDP_8BIT_PORT: u16 = 0xf000;
    pub const UDP_8BIT_PORT_MASK: u16 = 0xff00;

    pub const UDP_CHECKSUM_FLAG: u8 = 0b100;
    pub const UDP_SRC_PORT_FLAG: u8 = 0b010;
    pub const UDP_DST_PORT_FLAG: u8 = 0b001;
}

#[derive(Copy, Clone, Debug)]
pub struct Context {
    pub prefix: [u8; 16],
    pub prefix_len: u8,
    pub id: u8,
    pub compress: bool,
}

/// LoWPAN encoding requires being able to look up the existence of contexts,
/// which are essentially IPv6 address prefixes. Any implementation must ensure
/// that context 0 is always available and contains the mesh-local prefix.
pub trait ContextStore {
    fn get_context_from_addr(&self, ip_addr: IPAddr) -> Option<Context>;
    fn get_context_from_id(&self, ctx_id: u8) -> Option<Context>;
    fn get_context_0(&self) -> Context {
        match self.get_context_from_id(0) {
            Some(ctx) => ctx,
            None => panic!("Context 0 not found"),
        }
    }
    fn get_context_from_prefix(&self, prefix: &[u8], prefix_len: u8) -> Option<Context>;
}

/// Computes the LoWPAN Interface Identifier from either the 16-bit short MAC or
/// the IEEE EUI-64 that is derived from the 48-bit MAC.
pub fn compute_iid(mac_addr: &MacAddress) -> [u8; 8] {
    match mac_addr {
        &MacAddress::Short(short_addr) => {
            // IID is 0000:00ff:fe00:XXXX, where XXXX is 16-bit MAC
            let mut iid: [u8; 8] = iphc::MAC_BASE;
            iid[6] = (short_addr >> 1) as u8;
            iid[7] = (short_addr & 0xff) as u8;
            iid
        }
        &MacAddress::Long(long_addr) => {
            // IID is IEEE EUI-64 with universal/local bit inverted
            let mut iid: [u8; 8] = long_addr;
            iid[0] ^= iphc::MAC_UL;
            iid
        }
    }
}

impl ContextStore for Context {
    fn get_context_from_addr(&self, ip_addr: IPAddr) -> Option<Context> {
        if util::matches_prefix(&ip_addr.0, &self.prefix, self.prefix_len) {
            Some(*self)
        } else {
            None
        }
    }

    fn get_context_from_id(&self, ctx_id: u8) -> Option<Context> {
        if ctx_id == 0 {
            Some(*self)
        } else {
            None
        }
    }

    fn get_context_from_prefix(&self, prefix: &[u8], prefix_len: u8) -> Option<Context> {
        if prefix_len == self.prefix_len && util::matches_prefix(prefix, &self.prefix, prefix_len) {
            Some(*self)
        } else {
            None
        }
    }
}

pub fn is_lowpan(packet: &[u8]) -> bool {
    (packet[0] & iphc::DISPATCH[0]) == iphc::DISPATCH[0]
}

/// Determines if the next header is LoWPAN_NHC compressible, which depends on
/// both the next header type and the length of the IPv6 next header extensions.
/// Returns `Ok((false, 0))` if the next header is not compressible or
/// `Ok((true, nh_len))`. `nh_len` is only meaningful when the next header type
/// is an IPv6 next header extension, in which case it is the number of bytes
/// after the first two bytes in the IPv6 next header extension. Returns
/// `Err(())` in the case of an invalid IPv6 packet.
fn is_ip6_nh_compressible(next_header: u8, next_headers: &[u8]) -> Result<(bool, u8), ()> {
    match next_header {
        // IP6 encapsulated headers are always compressed
        ip6_nh::IP6 => Ok((true, 0)),
        // UDP headers are always compresed
        ip6_nh::UDP => Ok((true, 0)),
        ip6_nh::FRAGMENT
        | ip6_nh::HOP_OPTS
        | ip6_nh::ROUTING
        | ip6_nh::DST_OPTS
        | ip6_nh::MOBILITY => {
            let mut header_len: u32 = 6;
            if next_header != ip6_nh::FRAGMENT {
                // All compressible next header extensions except
                // for the fragment header have a length field
                if next_headers.len() < 2 {
                    return Err(());
                } else {
                    // The length field is the number of 8-octet
                    // groups after the first 8 octets
                    header_len += (next_headers[1] as u32) * 8;
                }
            }
            if header_len <= 255 {
                Ok((true, header_len as u8))
            } else {
                Ok((false, 0))
            }
        }
        _ => Ok((false, 0)),
    }
}

trait OnesComplement {
    fn ones_complement_add(self, other: Self) -> Self;
}

/// Implements one's complement addition for use in calculating the UDP checksum
impl OnesComplement for u16 {
    fn ones_complement_add(self, other: u16) -> u16 {
        let (sum, overflow) = self.overflowing_add(other);
        if overflow {
            sum + 1
        } else {
            sum
        }
    }
}

/// Computes the UDP checksum for a UDP packet sent over IPv6.
/// Returns the checksum in host byte-order.
fn compute_udp_checksum(
    ip6_header: &IP6Header,
    udp_header: &[u8],
    udp_length: u16,
    payload: &[u8],
) -> u16 {
    // The UDP checksum is computed on the IPv6 pseudo-header concatenated
    // with the UDP header and payload, but with the UDP checksum field
    // zeroed out. Hence, this function assumes that `udp_header` has already
    // been filled with the UDP header, except for the ignored checksum.
    let mut checksum: u16 = 0;

    // IPv6 pseudo-header
    // +--16 bits--+--16 bits--+--16 bits--+--16 bits--+
    // |                                               |
    // +              Source IPv6 Address              +
    // |                                               |
    // +-----------+-----------+-----------+-----------+
    // |                                               |
    // +           Destination IPv6 Address            +
    // |                                               |
    // +-----------+-----------+-----------+-----------+
    // |      UDP Length       |     0     |  NH type  |
    // +-----------+-----------+-----------+-----------+

    // Source and destination addresses
    for two_bytes in ip6_header.src_addr.0.chunks(2) {
        checksum = checksum.ones_complement_add(slice_to_u16(two_bytes));
    }
    for two_bytes in ip6_header.dst_addr.0.chunks(2) {
        checksum = checksum.ones_complement_add(slice_to_u16(two_bytes));
    }

    // UDP length and UDP next header type. Note that we can avoid adding zeros,
    // but the pseudo header must be in network byte-order.
    checksum = checksum.ones_complement_add(udp_length.to_be());
    checksum = checksum.ones_complement_add((ip6_nh::UDP as u16).to_be());

    // UDP header without the checksum (which is the last two bytes)
    for two_bytes in udp_header[0..6].chunks(2) {
        checksum = checksum.ones_complement_add(slice_to_u16(two_bytes));
    }

    // UDP payload
    for bytes in payload.chunks(2) {
        checksum = checksum.ones_complement_add(if bytes.len() == 2 {
            slice_to_u16(bytes)
        } else {
            (bytes[0] as u16).to_be()
        });
    }

    // Return the complement of the checksum, unless it is 0, in which case we
    // the checksum is one's complement -0 for a non-zero binary representation
    if !checksum != 0 {
        !checksum
    } else {
        checksum
    }
}

/// Maps values of a IPv6 next header field to a corresponding LoWPAN
/// NHC-encoding extension ID, if that next header type is NHC-compressible
fn ip6_nh_to_nhc_eid(next_header: u8) -> Option<u8> {
    match next_header {
        ip6_nh::HOP_OPTS => Some(nhc::HOP_OPTS),
        ip6_nh::ROUTING => Some(nhc::ROUTING),
        ip6_nh::FRAGMENT => Some(nhc::FRAGMENT),
        ip6_nh::DST_OPTS => Some(nhc::DST_OPTS),
        ip6_nh::MOBILITY => Some(nhc::MOBILITY),
        ip6_nh::IP6 => Some(nhc::IP6),
        _ => None,
    }
}

/// Maps a LoWPAN_NHC header the corresponding IPv6 next header type,
/// or an error if the NHC header is invalid
fn nhc_to_ip6_nh(nhc: u8) -> Result<u8, ()> {
    match nhc & nhc::DISPATCH_MASK {
        nhc::DISPATCH_NHC => match nhc & nhc::EID_MASK {
            nhc::HOP_OPTS => Ok(ip6_nh::HOP_OPTS),
            nhc::ROUTING => Ok(ip6_nh::ROUTING),
            nhc::FRAGMENT => Ok(ip6_nh::FRAGMENT),
            nhc::DST_OPTS => Ok(ip6_nh::DST_OPTS),
            nhc::MOBILITY => Ok(ip6_nh::MOBILITY),
            nhc::IP6 => Ok(ip6_nh::IP6),
            _ => Err(()),
        },
        nhc::DISPATCH_UDP => Ok(ip6_nh::UDP),
        _ => Err(()),
    }
}

/// Compresses an IPv6 header into a 6loWPAN header
///
/// Constructs a 6LoWPAN header in `buf` from the given IPv6 datagram and
/// 16-bit MAC addresses. If the compression was successful, returns
/// `Ok((consumed, written))`, where `consumed` is the number of header
/// bytes consumed from the IPv6 datagram `written` is the number of
/// compressed header bytes written into `buf`. Payload bytes and
/// non-compressed next headers are not written, so the remaining `buf.len()
/// - consumed` bytes must still be copied over to `buf`.
pub fn compress(
    ctx_store: &ContextStore,
    ip6_datagram: &[u8],
    src_mac_addr: MacAddress,
    dst_mac_addr: MacAddress,
    mut buf: &mut [u8],
) -> Result<(usize, usize), ()> {
    // Note that consumed should be constant, and equal sizeof(IP6Header)
    let (mut consumed, ip6_header) = IP6Header::decode(ip6_datagram).done().ok_or(())?;
    let mut next_headers: &[u8] = &ip6_datagram[consumed..];

    // The first two bytes are the LOWPAN_IPHC header
    let mut written: usize = 2;

    // Initialize the LOWPAN_IPHC header
    buf[0..2].copy_from_slice(&iphc::DISPATCH);

    let mut src_ctx: Option<Context> = ctx_store.get_context_from_addr(ip6_header.src_addr);
    let mut dst_ctx: Option<Context> = if ip6_header.dst_addr.is_multicast() {
        let prefix_len: u8 = ip6_header.dst_addr.0[3];
        let prefix: &[u8] = &ip6_header.dst_addr.0[4..12];
        // This also implicitly verifies that prefix_len <= 64
        if util::verify_prefix_len(prefix, prefix_len) {
            ctx_store.get_context_from_prefix(prefix, prefix_len)
        } else {
            None
        }
    } else {
        ctx_store.get_context_from_addr(ip6_header.dst_addr)
    };

    // Do not contexts that are not marked to be available for compression
    src_ctx = src_ctx.and_then(|ctx| if ctx.compress { Some(ctx) } else { None });
    dst_ctx = dst_ctx.and_then(|ctx| if ctx.compress { Some(ctx) } else { None });

    // Context Identifier Extension
    compress_cie(&src_ctx, &dst_ctx, &mut buf, &mut written);

    // Traffic Class & Flow Label
    compress_tf(&ip6_header, &mut buf, &mut written);

    // Next Header
    let (mut is_nhc, mut nh_len): (bool, u8) =
        is_ip6_nh_compressible(ip6_header.next_header, next_headers)?;
    compress_nh(&ip6_header, is_nhc, &mut buf, &mut written);

    // Hop Limit
    compress_hl(&ip6_header, &mut buf, &mut written);

    // Source Address
    compress_src(
        &ip6_header.src_addr,
        &src_mac_addr,
        &src_ctx,
        &mut buf,
        &mut written,
    );

    // Destination Address
    if ip6_header.dst_addr.is_multicast() {
        compress_multicast(&ip6_header.dst_addr, &dst_ctx, &mut buf, &mut written);
    } else {
        compress_dst(
            &ip6_header.dst_addr,
            &dst_mac_addr,
            &dst_ctx,
            &mut buf,
            &mut written,
        );
    }

    // Next Headers
    // At each iteration, next_headers begins at the first byte of the
    // current uncompressed next header.
    let mut ip6_nh_type: u8 = ip6_header.next_header;
    while is_nhc {
        match ip6_nh_type {
            ip6_nh::IP6 => {
                // For IPv6 encapsulation, the NH bit in the NHC ID is 0
                let nhc_header = nhc::DISPATCH_NHC | nhc::IP6;
                buf[written] = nhc_header;
                written += 1;

                // Recursively place IPHC-encoded IPv6 after the NHC ID
                let (encap_consumed, encap_written) = compress(
                    ctx_store,
                    next_headers,
                    src_mac_addr,
                    dst_mac_addr,
                    &mut buf[written..],
                )?;
                consumed += encap_consumed;
                written += encap_written;

                // The above recursion handles the rest of the packet
                // headers, so we are done
                break;
            }
            ip6_nh::UDP => {
                let mut nhc_header = nhc::DISPATCH_UDP;

                // Leave a space for the UDP LoWPAN_NHC byte
                let udp_nh_offset = written;
                written += 1;

                // Compress ports and checksum
                let udp_header = &next_headers[0..8];
                nhc_header |= compress_udp_ports(udp_header, &mut buf, &mut written);
                nhc_header |= compress_udp_checksum(udp_header, &mut buf, &mut written);

                // Write the UDP LoWPAN_NHC byte
                buf[udp_nh_offset] = nhc_header;
                consumed += 8;

                // There cannot be any more next headers after UDP
                break;
            }
            ip6_nh::FRAGMENT
            | ip6_nh::HOP_OPTS
            | ip6_nh::ROUTING
            | ip6_nh::DST_OPTS
            | ip6_nh::MOBILITY => {
                // is_ip6_nh_compressible guarantees that the IPv6 next
                // header corresponds to a valid LoWPAN_NHC EID
                let mut nhc_header = nhc::DISPATCH_NHC | match ip6_nh_to_nhc_eid(ip6_nh_type) {
                    Some(eid) => eid,
                    None => panic!("Unreachable case"),
                };

                // next_nh_offset includes the next header field and the
                // length byte, while nh_len does not
                let next_nh_offset = 2 + (nh_len as usize);

                // Determine if the next header is compressible
                let (next_is_nhc, next_nh_len) =
                    is_ip6_nh_compressible(next_headers[0], &next_headers[next_nh_offset..])?;
                if next_is_nhc {
                    nhc_header |= nhc::NH;
                }

                // Place NHC ID in buffer
                buf[written] = nhc_header;
                if ip6_nh_type != ip6_nh::FRAGMENT {
                    // Fragment extension does not have a length field
                    buf[written + 1] = nh_len;
                }
                written += 2;

                compress_and_elide_padding(
                    ip6_nh_type,
                    nh_len as usize,
                    &next_headers,
                    &mut buf,
                    &mut written,
                );

                ip6_nh_type = next_headers[0];
                is_nhc = next_is_nhc;
                nh_len = next_nh_len;
                next_headers = &next_headers[next_nh_offset..];
                consumed += next_nh_offset;
            }
            // This case should not be reachable because
            // is_ip6_nh_compressible guarantees that is_nhc is true
            // only if ip6_nh_type is one of the types matched above
            _ => panic!("Unreachable case"),
        }
    }
    Ok((consumed, written))
}

fn compress_cie(
    src_ctx: &Option<Context>,
    dst_ctx: &Option<Context>,
    buf: &mut [u8],
    written: &mut usize,
) {
    let mut cie: u8 = 0;

    src_ctx.as_ref().map(|ctx| {
        if ctx.id != 0 {
            cie |= ctx.id << 4;
        }
    });
    dst_ctx.as_ref().map(|ctx| {
        if ctx.id != 0 {
            cie |= ctx.id;
        }
    });

    if cie != 0 {
        buf[1] |= iphc::CID;
        buf[*written] = cie;
        *written += 1;
    }
}

fn compress_tf(ip6_header: &IP6Header, buf: &mut [u8], written: &mut usize) {
    let ecn = ip6_header.get_ecn();
    let dscp = ip6_header.get_dscp();
    let flow = ip6_header.get_flow_label();

    let mut tf_encoding = 0;
    let old_offset = *written;

    // If ECN != 0 we are forced to at least have one byte,
    // otherwise we can elide dscp
    if dscp == 0 && (ecn == 0 || flow != 0) {
        tf_encoding |= iphc::TF_TRAFFIC_CLASS;
    } else {
        buf[*written] = dscp;
        *written += 1;
    }

    // We can elide flow if it is 0
    if flow == 0 {
        tf_encoding |= iphc::TF_FLOW_LABEL;
    } else {
        buf[*written] = ((flow >> 16) & 0x0f) as u8;
        buf[*written + 1] = (flow >> 8) as u8;
        buf[*written + 2] = flow as u8;
        *written += 3;
    }

    if *written != old_offset {
        buf[old_offset] |= ecn << 6;
    }
    buf[0] |= tf_encoding;
}

fn compress_nh(ip6_header: &IP6Header, is_nhc: bool, buf: &mut [u8], written: &mut usize) {
    if is_nhc {
        buf[0] |= iphc::NH;
    } else {
        buf[*written] = ip6_header.next_header;
        *written += 1;
    }
}

fn compress_hl(ip6_header: &IP6Header, buf: &mut [u8], written: &mut usize) {
    let hop_limit_flag = match ip6_header.hop_limit {
        1 => iphc::HLIM_1,
        64 => iphc::HLIM_64,
        255 => iphc::HLIM_255,
        _ => {
            buf[*written] = ip6_header.hop_limit;
            *written += 1;
            iphc::HLIM_INLINE
        }
    };
    buf[0] |= hop_limit_flag;
}

// TODO: We should check to see whether context or link local compression
// schemes gives the better compression; currently, we will always match
// on link local even if we could get better compression through context.
fn compress_src(
    src_ip_addr: &IPAddr,
    src_mac_addr: &MacAddress,
    src_ctx: &Option<Context>,
    buf: &mut [u8],
    written: &mut usize,
) {
    if src_ip_addr.is_unspecified() {
        // SAC = 1, SAM = 00
        buf[1] |= iphc::SAC;
    } else if src_ip_addr.is_unicast_link_local() {
        // SAC = 0, SAM = 01, 10, 11
        compress_iid(src_ip_addr, src_mac_addr, true, buf, written);
    } else if src_ctx.is_some() {
        // SAC = 1, SAM = 01, 10, 11
        buf[1] |= iphc::SAC;
        compress_iid(src_ip_addr, src_mac_addr, true, buf, written);
    } else {
        // SAC = 0, SAM = 00
        buf[*written..*written + 16].copy_from_slice(&src_ip_addr.0);
        *written += 16;
    }
}

// TODO: For the SAC = 0, SAM = 11 case in IPv6-encapsulated headers,
// it might be that we have to compute the IID from the encapsulating
// IPv6 header address instead of the EUI-64 from the 802.15.4 layer
fn compress_iid(
    ip_addr: &IPAddr,
    mac_addr: &MacAddress,
    is_src: bool,
    buf: &mut [u8],
    written: &mut usize,
) {
    let iid: [u8; 8] = compute_iid(mac_addr);
    if ip_addr.0[8..16] == iid {
        // SAM/DAM = 11, 0 bits
        buf[1] |= if is_src {
            iphc::SAM_MODE3
        } else {
            iphc::DAM_MODE3
        };
    } else if ip_addr.0[8..14] == iphc::MAC_BASE[0..6] {
        // SAM/DAM = 10, 16 bits
        buf[1] |= if is_src {
            iphc::SAM_MODE2
        } else {
            iphc::DAM_MODE2
        };
        buf[*written..*written + 2].copy_from_slice(&ip_addr.0[14..16]);
        *written += 2;
    } else {
        // SAM/DAM = 01, 64 bits
        buf[1] |= if is_src {
            iphc::SAM_MODE1
        } else {
            iphc::DAM_MODE1
        };
        buf[*written..*written + 8].copy_from_slice(&ip_addr.0[8..16]);
        *written += 8;
    }
}

// Compresses non-multicast destination address
// TODO: We should check to see whether context or link local compression
// schemes gives the better compression; currently, we will always match
// on link local even if we could get better compression through context.
fn compress_dst(
    dst_ip_addr: &IPAddr,
    dst_mac_addr: &MacAddress,
    dst_ctx: &Option<Context>,
    buf: &mut [u8],
    written: &mut usize,
) {
    // Assumes dst_ip_addr is not a multicast address (prefix ffXX)
    if dst_ip_addr.is_unicast_link_local() {
        // Link local compression
        // M = 0, DAC = 0, DAM = 01, 10, 11
        compress_iid(dst_ip_addr, dst_mac_addr, false, buf, written);
    } else if dst_ctx.is_some() {
        // Context compression
        // DAC = 1, DAM = 01, 10, 11
        buf[1] |= iphc::DAC;
        compress_iid(dst_ip_addr, dst_mac_addr, false, buf, written);
    } else {
        // Full address inline
        // DAC = 0, DAM = 00
        buf[*written..*written + 16].copy_from_slice(&dst_ip_addr.0);
        *written += 16;
    }
}

// Compresses multicast destination addresses
fn compress_multicast(
    dst_ip_addr: &IPAddr,
    dst_ctx: &Option<Context>,
    buf: &mut [u8],
    written: &mut usize,
) {
    // Assumes dst_ip_addr is indeed a multicast address (prefix ffXX)
    buf[1] |= iphc::MULTICAST;
    if dst_ctx.is_some() {
        // M = 1, DAC = 1, DAM = 00
        buf[1] |= iphc::DAC;
        buf[*written..*written + 2].copy_from_slice(&dst_ip_addr.0[1..3]);
        buf[*written + 2..*written + 6].copy_from_slice(&dst_ip_addr.0[12..16]);
        *written += 6;
    } else {
        // M = 1, DAC = 0
        if dst_ip_addr.0[1] == 0x02 && dst_ip_addr.0[2..15].iter().all(|&b| b == 0) {
            // DAM = 11
            buf[1] |= iphc::DAM_MODE3;
            buf[*written] = dst_ip_addr.0[15];
            *written += 1;
        } else {
            if !dst_ip_addr.0[2..11].iter().all(|&b| b == 0) {
                // DAM = 00
                buf[1] |= iphc::DAM_INLINE;
                buf[*written..*written + 16].copy_from_slice(&dst_ip_addr.0);
                *written += 16;
            } else if !dst_ip_addr.0[11..13].iter().all(|&b| b == 0) {
                // DAM = 01, ffXX::00XX:XXXX:XXXX
                buf[1] |= iphc::DAM_MODE1;
                buf[*written] = dst_ip_addr.0[1];
                buf[*written + 1..*written + 6].copy_from_slice(&dst_ip_addr.0[11..16]);
                *written += 6;
            } else {
                // DAM = 10, ffXX::00XX:XXXX
                buf[1] |= iphc::DAM_MODE2;
                buf[*written] = dst_ip_addr.0[1];
                buf[*written + 1..*written + 4].copy_from_slice(&dst_ip_addr.0[13..16]);
                *written += 4;
            }
        }
    }
}

fn compress_udp_ports(udp_header: &[u8], buf: &mut [u8], written: &mut usize) -> u8 {
    let src_port = u16::from_be(slice_to_u16(&udp_header[0..2]));
    let dst_port = u16::from_be(slice_to_u16(&udp_header[2..4]));

    let mut udp_port_nhc = 0;
    if (src_port & nhc::UDP_4BIT_PORT_MASK) == nhc::UDP_4BIT_PORT
        && (dst_port & nhc::UDP_4BIT_PORT_MASK) == nhc::UDP_4BIT_PORT
    {
        // Both can be compressed to 4 bits
        udp_port_nhc |= nhc::UDP_SRC_PORT_FLAG | nhc::UDP_DST_PORT_FLAG;
        // This should compress the ports to a single 8-bit value,
        // with the source port before the destination port
        buf[*written] = (((src_port & !nhc::UDP_4BIT_PORT_MASK) << 4)
            | (dst_port & !nhc::UDP_4BIT_PORT_MASK)) as u8;
        *written += 1;
    } else if (src_port & nhc::UDP_8BIT_PORT_MASK) == nhc::UDP_8BIT_PORT {
        // Source port compressed to 8 bits, destination port uncompressed
        udp_port_nhc |= nhc::UDP_SRC_PORT_FLAG;
        buf[*written] = (src_port & !nhc::UDP_8BIT_PORT_MASK) as u8;
        u16_to_slice(dst_port.to_be(), &mut buf[*written + 1..*written + 3]);
        *written += 3;
    } else if (dst_port & nhc::UDP_8BIT_PORT_MASK) == nhc::UDP_8BIT_PORT {
        udp_port_nhc |= nhc::UDP_DST_PORT_FLAG;
        u16_to_slice(src_port.to_be(), &mut buf[*written..*written + 2]);
        buf[*written + 3] = (dst_port & !nhc::UDP_8BIT_PORT_MASK) as u8;
        *written += 3;
    } else {
        buf[*written..*written + 4].copy_from_slice(&udp_header[0..4]);
        *written += 4;
    }
    return udp_port_nhc;
}

fn compress_udp_checksum(udp_header: &[u8], buf: &mut [u8], written: &mut usize) -> u8 {
    // TODO: Checksum is always inline, elision is currently not supported
    buf[*written] = udp_header[6];
    buf[*written + 1] = udp_header[7];
    *written += 2;
    // Inline checksum corresponds to the 0 flag
    0
}

fn compress_and_elide_padding(
    nh_type: u8,
    nh_len: usize,
    next_headers: &[u8],
    buf: &mut [u8],
    written: &mut usize,
) {
    let total_len = nh_len + 2;
    // is_multiple is true if the header length is a multiple of 8-octets
    let is_multiple = (total_len % 8) == 0;
    let correct_type = (nh_type == ip6_nh::HOP_OPTS) || (nh_type == ip6_nh::DST_OPTS);
    let mut opt_offset = 2;
    let mut is_padding = false;
    if correct_type && is_multiple {
        // Traverses the TLVs in the next header extension. We need to
        // determine if there is a last padding TLV (Pad1 or PadN) that is
        // not preceded by another padding TLV. Hence, we have a state
        // machine that keeps track of whether the last TLV was a padding
        // TLV. We only set is_padding to true if we encounter a padding
        // byte that spans to the end of the TLV chain, and if the previous
        // TLV was not a padding TLV. In that case, we break out of the loop
        // so that opt_offset is the offset before the last padding TLV.
        let mut prev_was_padding = false;
        while opt_offset < total_len {
            let opt_type = next_headers[opt_offset];
            let new_opt_offset = match opt_type {
                // Pad1, PadN
                0 | 1 => {
                    let new_opt_offset = match opt_type {
                        0 => opt_offset + 1,
                        1 => {
                            let opt_len = next_headers[opt_offset + 1] as usize;
                            opt_offset + opt_len + 2
                        }
                        _ => panic!("Unreachable case"),
                    };
                    if new_opt_offset == total_len {
                        if !prev_was_padding {
                            is_padding = true;
                        }
                        break;
                    }
                    prev_was_padding = true;
                    new_opt_offset
                }
                // Any other TLV type
                _ => {
                    let opt_len = next_headers[opt_offset + 1] as usize;
                    prev_was_padding = false;
                    opt_offset + opt_len + 2
                }
            };
            opt_offset = new_opt_offset;
        }
    }

    // We only elide the padding if: 1) Encapsulating packet is a multiple
    // of 8 octets in length, 2) the header is either hop options or dest
    // options, and 3) if there is a single Pad1 or PadN trailing padding.
    if is_multiple && correct_type && is_padding {
        buf[*written..*written + opt_offset - 2].copy_from_slice(&next_headers[2..opt_offset]);
        *written += opt_offset - 2;
    } else {
        // Copy over the remaining packet data
        buf[*written..*written + nh_len].copy_from_slice(&next_headers[2..2 + nh_len]);
        *written += nh_len;
    }
}

/// Decompresses a 6loWPAN header into a full IPv6 header
///
/// This function decompresses the header found in `buf` and writes it
/// `out_buf`. It does not, though, copy payload bytes or a non-compressed next
/// header. As a result, the caller should copy the `buf.len - consumed`
/// remaining bytes from `buf` to `out_buf`.
///
///
/// Note that in the case of fragmentation, the total length of the IPv6
/// packet cannot be inferred from a single frame, and is instead provided
/// by the dgram_size field in the fragmentation header. Thus, if we are
/// decompressing a fragment, we rely on the dgram_size field; otherwise,
/// we infer the length from the size of buf.
///
/// # Arguments
///
/// * `ctx_store` - ???
///
/// * `buf` - A slice containing the 6LowPAN packet along with its payload.
///
/// * `src_mac_addr` - the 16-bit MAC address of the frame sender.
///
/// * `dst_mac_addr` - the 16-bit MAC address of the frame receiver.
///
/// * `out_buf` - A buffer to write the output to. Must be at least large enough
/// to store an IPv6 header (XX bytes).
///
/// * `dgram_size` - If `is_fragment` is `true`, this is used as the IPv6
/// packets total payload size. Otherwise, this is ignored.
///
/// * `is_fragment` - ???
///
/// # Returns
///
/// `Ok((consumed, written))` if decompression is successful.
///
/// * `consumed` is the number of header bytes consumed from the 6LoWPAN header
///
/// * `written` is the number of uncompressed header bytes written into
/// `out_buf`.
pub fn decompress(
    ctx_store: &ContextStore,
    buf: &[u8],
    src_mac_addr: MacAddress,
    dst_mac_addr: MacAddress,
    out_buf: &mut [u8],
    dgram_size: u16,
    is_fragment: bool,
) -> Result<(usize, usize), ()> {
    // Get the LOWPAN_IPHC header (the first two bytes are the header)
    let iphc_header_1: u8 = buf[0];
    let iphc_header_2: u8 = buf[1];
    let mut consumed: usize = 2;

    let mut ip6_header = IP6Header::new();
    let mut written: usize = mem::size_of::<IP6Header>();

    // Decompress CID and CIE fields if they exist
    let (src_ctx, dst_ctx) = decompress_cie(ctx_store, iphc_header_1, &buf, &mut consumed)?;

    // Traffic Class & Flow Label
    decompress_tf(&mut ip6_header, iphc_header_1, &buf, &mut consumed);

    // Next Header
    let (mut is_nhc, mut next_header) = decompress_nh(iphc_header_1, &buf, &mut consumed);

    // Hop Limit
    decompress_hl(&mut ip6_header, iphc_header_1, &buf, &mut consumed)?;

    // Source Address
    decompress_src(
        &mut ip6_header,
        iphc_header_2,
        &src_mac_addr,
        &src_ctx,
        &buf,
        &mut consumed,
    )?;

    // Destination Address
    if (iphc_header_2 & iphc::MULTICAST) != 0 {
        decompress_multicast(
            &mut ip6_header,
            iphc_header_2,
            &dst_ctx,
            &buf,
            &mut consumed,
        )?;
    } else {
        decompress_dst(
            &mut ip6_header,
            iphc_header_2,
            &dst_mac_addr,
            &dst_ctx,
            &buf,
            &mut consumed,
        )?;
    }

    // next_header is already set if is_nhc is false, otherwise it can be
    // determined from the LoWPAN NHC header byte
    if is_nhc {
        next_header = nhc_to_ip6_nh(buf[consumed])?;
    }
    ip6_header.set_next_header(next_header);

    // Next headers after the IPv6 fixed header
    // At each iteration, consumed points to the first byte of the compressed
    // next header in buf.
    while is_nhc {
        // Advance past the LoWPAN NHC byte
        let nhc_header = buf[consumed];
        consumed += 1;

        // Scoped mutable borrow of out_buf
        let mut next_headers: &mut [u8] = &mut out_buf[written..];

        match next_header {
            ip6_nh::IP6 => {
                let (encap_consumed, encap_written) = decompress(
                    ctx_store,
                    &buf[consumed..],
                    src_mac_addr,
                    dst_mac_addr,
                    &mut next_headers,
                    dgram_size,
                    is_fragment,
                )?;
                consumed += encap_consumed;
                written += encap_written;
                break;
            }
            ip6_nh::UDP => {
                // UDP length includes UDP header and data in bytes
                let udp_length = (8 + (buf.len() - consumed)) as u16;
                // Decompress UDP header fields
                let (src_port, dst_port) = decompress_udp_ports(nhc_header, &buf, &mut consumed);
                // Fill in uncompressed UDP header
                u16_to_slice(src_port.to_be(), &mut next_headers[0..2]);
                u16_to_slice(dst_port.to_be(), &mut next_headers[2..4]);
                u16_to_slice(udp_length.to_be(), &mut next_headers[4..6]);
                // Need to fill in header values before computing the checksum
                let udp_checksum = decompress_udp_checksum(
                    nhc_header,
                    &next_headers[0..8],
                    udp_length,
                    &ip6_header,
                    &buf,
                    &mut consumed,
                );
                u16_to_slice(udp_checksum.to_be(), &mut next_headers[6..8]);

                written += 8;
                break;
            }
            ip6_nh::FRAGMENT
            | ip6_nh::HOP_OPTS
            | ip6_nh::ROUTING
            | ip6_nh::DST_OPTS
            | ip6_nh::MOBILITY => {
                // True if the next header is also compressed
                is_nhc = (nhc_header & nhc::NH) != 0;

                // len is the number of octets following the length field
                let len = buf[consumed] as usize;
                consumed += 1;

                // Check that there is a next header in the buffer,
                // which must be the case if the last next header specifies
                // NH = 1
                if consumed + len >= buf.len() {
                    return Err(());
                }

                // Length in 8-octet units after the first 8 octets
                // (per the IPv6 ext hdr spec)
                let mut hdr_len_field = (len - 6) / 8;
                if (len - 6) % 8 != 0 {
                    hdr_len_field += 1;
                }

                // Gets the type of the subsequent next header.  If is_nhc
                // is true, there must be a LoWPAN NHC header byte,
                // otherwise there is either an uncompressed next header.
                next_header = if is_nhc {
                    // The next header is LoWPAN NHC-compressed
                    nhc_to_ip6_nh(buf[consumed + len])?
                } else {
                    // The next header is uncompressed
                    buf[consumed + len]
                };

                // Fill in the extended header in uncompressed IPv6 format
                next_headers[0] = next_header;
                next_headers[1] = hdr_len_field as u8;
                // Copies over the remaining options.
                next_headers[2..2 + len].copy_from_slice(&buf[consumed..consumed + len]);

                // Fill in padding
                let pad_bytes = hdr_len_field * 8 - len + 6;
                if pad_bytes == 1 {
                    // Pad1
                    next_headers[2 + len] = 0;
                } else {
                    // PadN, 2 <= pad_bytes <= 7
                    next_headers[2 + len] = 1;
                    next_headers[2 + len + 1] = pad_bytes as u8 - 2;
                    for i in 2..pad_bytes {
                        next_headers[2 + len + i] = 0;
                    }
                }

                written += 8 + hdr_len_field * 8;
                consumed += len;
            }
            _ => panic!("Unreachable case"),
        }
    }

    // The IPv6 header length field is the size of the IPv6 payload,
    // including extension headers. This is thus the uncompressed
    // size of the IPv6 packet - the fixed IPv6 header.
    let payload_len = if is_fragment {
        (dgram_size as usize) - mem::size_of::<IP6Header>()
    } else {
        written + (buf.len() - consumed) - mem::size_of::<IP6Header>()
    };
    ip6_header.payload_len = (payload_len as u16).to_be();
    IP6Header::encode(out_buf, ip6_header).done().ok_or(())?;
    Ok((consumed, written))
}

fn decompress_cie(
    ctx_store: &ContextStore,
    iphc_header: u8,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(Context, Context), ()> {
    let ctx_0 = ctx_store.get_context_0();
    let (mut src_ctx, mut dst_ctx) = (ctx_0, ctx_0);
    if iphc_header & iphc::CID != 0 {
        let sci = buf[*consumed] >> 4;
        let dci = buf[*consumed] & 0xf;
        *consumed += 1;

        if sci != 0 {
            src_ctx = ctx_store.get_context_from_id(sci).ok_or(())?;
        }
        if dci != 0 {
            dst_ctx = ctx_store.get_context_from_id(dci).ok_or(())?;
        }
    }
    Ok((src_ctx, dst_ctx))
}

fn decompress_tf(ip6_header: &mut IP6Header, iphc_header: u8, buf: &[u8], consumed: &mut usize) {
    let fl_compressed = (iphc_header & iphc::TF_FLOW_LABEL) != 0;
    let tc_compressed = (iphc_header & iphc::TF_TRAFFIC_CLASS) != 0;

    // Determine ECN and DSCP separately because the order is different
    // from the IPv6 traffic class field.
    if !fl_compressed || !tc_compressed {
        let ecn = buf[*consumed] >> 6;
        ip6_header.set_ecn(ecn);
    }
    if !tc_compressed {
        let dscp = buf[*consumed] & 0b111111;
        ip6_header.set_dscp(dscp);
        *consumed += 1;
    }

    // Flow label is always in the same bit position relative to the last
    // three bytes in the inline fields
    if fl_compressed {
        ip6_header.set_flow_label(0);
    } else {
        let flow = (((buf[*consumed] & 0x0f) as u32) << 16) | ((buf[*consumed + 1] as u32) << 8)
            | (buf[*consumed + 2] as u32);
        *consumed += 3;
        ip6_header.set_flow_label(flow);
    }
}

fn decompress_nh(iphc_header: u8, buf: &[u8], consumed: &mut usize) -> (bool, u8) {
    let is_nhc = (iphc_header & iphc::NH) != 0;
    let mut next_header: u8 = 0;
    if !is_nhc {
        next_header = buf[*consumed];
        *consumed += 1;
    }
    return (is_nhc, next_header);
}

fn decompress_hl(
    ip6_header: &mut IP6Header,
    iphc_header: u8,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let hop_limit = match iphc_header & iphc::HLIM_MASK {
        iphc::HLIM_1 => 1,
        iphc::HLIM_64 => 64,
        iphc::HLIM_255 => 255,
        iphc::HLIM_INLINE => {
            let hl = buf[*consumed];
            *consumed += 1;
            hl
        }
        _ => panic!("Unreachable case"),
    };
    ip6_header.set_hop_limit(hop_limit);
    Ok(())
}

fn decompress_src(
    ip6_header: &mut IP6Header,
    iphc_header: u8,
    mac_addr: &MacAddress,
    ctx: &Context,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let uses_context = (iphc_header & iphc::SAC) != 0;
    let sam_mode = iphc_header & iphc::SAM_MASK;
    if uses_context && sam_mode == iphc::SAM_INLINE {
        // SAC = 1, SAM = 00: UNSPECIFIED (::), which is already the default
    } else if uses_context {
        // SAC = 1, SAM = 01, 10, 11
        decompress_iid_context(
            sam_mode,
            &mut ip6_header.src_addr,
            mac_addr,
            ctx,
            buf,
            consumed,
        )?;
    } else {
        // SAC = 0, SAM = 00, 01, 10, 11
        decompress_iid_link_local(sam_mode, &mut ip6_header.src_addr, mac_addr, buf, consumed)?;
    }
    Ok(())
}

fn decompress_dst(
    ip6_header: &mut IP6Header,
    iphc_header: u8,
    mac_addr: &MacAddress,
    ctx: &Context,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let uses_context = (iphc_header & iphc::DAC) != 0;
    let dam_mode = iphc_header & iphc::DAM_MASK;
    if uses_context && dam_mode == iphc::DAM_INLINE {
        // DAC = 1, DAM = 00: Reserved
        return Err(());
    } else if uses_context {
        // DAC = 1, DAM = 01, 10, 11
        decompress_iid_context(
            dam_mode,
            &mut ip6_header.dst_addr,
            mac_addr,
            ctx,
            buf,
            consumed,
        )?;
    } else {
        // DAC = 0, DAM = 00, 01, 10, 11
        decompress_iid_link_local(dam_mode, &mut ip6_header.dst_addr, mac_addr, buf, consumed)?;
    }
    Ok(())
}

fn decompress_multicast(
    ip6_header: &mut IP6Header,
    iphc_header: u8,
    ctx: &Context,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let uses_context = (iphc_header & iphc::DAC) != 0;
    let dam_mode = iphc_header & iphc::DAM_MASK;
    let ip_addr: &mut IPAddr = &mut ip6_header.dst_addr;
    if uses_context {
        match dam_mode {
            iphc::DAM_INLINE => {
                // DAC = 1, DAM = 00: 48 bits
                // ffXX:XXLL:PPPP:PPPP:PPPP:PPPP:XXXX:XXXX
                let prefix_bytes = ((ctx.prefix_len + 7) / 8) as usize;
                if prefix_bytes > 8 {
                    // The maximum prefix length for this mode is 64 bits.
                    // If the specified prefix exceeds this length, the
                    // compression is invalid.
                    return Err(());
                }
                ip_addr.0[0] = 0xff;
                ip_addr.0[1] = buf[*consumed];
                ip_addr.0[2] = buf[*consumed + 1];
                ip_addr.0[3] = ctx.prefix_len;
                ip_addr.0[4..4 + prefix_bytes].copy_from_slice(&ctx.prefix[0..prefix_bytes]);
                ip_addr.0[12..16].copy_from_slice(&buf[*consumed + 2..*consumed + 6]);
                *consumed += 6;
            }
            _ => {
                // DAC = 1, DAM = 01, 10, 11: Reserved
                return Err(());
            }
        }
    } else {
        match dam_mode {
            // DAC = 0, DAM = 00: Inline
            iphc::DAM_INLINE => {
                ip_addr.0.copy_from_slice(&buf[*consumed..*consumed + 16]);
                *consumed += 16;
            }
            // DAC = 0, DAM = 01: 48 bits
            // ffXX::00XX:XXXX:XXXX
            iphc::DAM_MODE1 => {
                ip_addr.0[0] = 0xff;
                ip_addr.0[1] = buf[*consumed];
                *consumed += 1;
                ip_addr.0[11..16].copy_from_slice(&buf[*consumed..*consumed + 5]);
                *consumed += 5;
            }
            // DAC = 0, DAM = 10: 32 bits
            // ffXX::00XX:XXXX
            iphc::DAM_MODE2 => {
                ip_addr.0[0] = 0xff;
                ip_addr.0[1] = buf[*consumed];
                *consumed += 1;
                ip_addr.0[13..16].copy_from_slice(&buf[*consumed..*consumed + 3]);
                *consumed += 3;
            }
            // DAC = 0, DAM = 11: 8 bits
            // ff02::00XX
            iphc::DAM_MODE3 => {
                ip_addr.0[0] = 0xff;
                ip_addr.0[1] = 0x02;
                ip_addr.0[15] = buf[*consumed];
                *consumed += 1;
            }
            _ => panic!("Unreachable case"),
        }
    }
    Ok(())
}

fn decompress_iid_link_local(
    addr_mode: u8,
    ip_addr: &mut IPAddr,
    mac_addr: &MacAddress,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let mode = addr_mode & (iphc::SAM_MASK | iphc::DAM_MASK);
    match mode {
        // SAM, DAM = 00: Inline
        iphc::SAM_INLINE => {
            // SAM_INLINE is equivalent to DAM_INLINE
            ip_addr.0.copy_from_slice(&buf[*consumed..*consumed + 16]);
            *consumed += 16;
        }
        // SAM, DAM = 01: 64 bits
        // Link-local prefix (64 bits) + 64 bits carried inline
        iphc::SAM_MODE1 | iphc::DAM_MODE1 => {
            ip_addr.set_unicast_link_local();
            ip_addr.0[8..16].copy_from_slice(&buf[*consumed..*consumed + 8]);
            *consumed += 8;
        }
        // SAM, DAM = 11: 16 bits
        // Link-local prefix (112 bits) + 0000:00ff:fe00:XXXX
        iphc::SAM_MODE2 | iphc::DAM_MODE2 => {
            ip_addr.set_unicast_link_local();
            ip_addr.0[11..13].copy_from_slice(&iphc::MAC_BASE[3..5]);
            ip_addr.0[14..16].copy_from_slice(&buf[*consumed..*consumed + 2]);
            *consumed += 2;
        }
        // SAM, DAM = 11: 0 bits
        // Linx-local prefix (64 bits) + IID from outer header (64 bits)
        iphc::SAM_MODE3 | iphc::DAM_MODE3 => {
            ip_addr.set_unicast_link_local();
            ip_addr.0[8..16].copy_from_slice(&compute_iid(mac_addr));
        }
        _ => panic!("Unreachable case"),
    }
    Ok(())
}

fn decompress_iid_context(
    addr_mode: u8,
    ip_addr: &mut IPAddr,
    mac_addr: &MacAddress,
    ctx: &Context,
    buf: &[u8],
    consumed: &mut usize,
) -> Result<(), ()> {
    let mode = addr_mode & (iphc::SAM_MASK | iphc::DAM_MASK);
    match mode {
        // DAM = 00: Reserved
        // SAM = 0 is handled separately outside this method
        iphc::DAM_INLINE => {
            return Err(());
        }
        // SAM, DAM = 01: 64 bits
        // Suffix is the 64 bits carried inline
        iphc::SAM_MODE1 | iphc::DAM_MODE1 => {
            ip_addr.0[8..16].copy_from_slice(&buf[*consumed..*consumed + 8]);
            *consumed += 8;
        }
        // SAM, DAM = 10: 16 bits
        // Suffix is 0000:00ff:fe00:XXXX
        iphc::SAM_MODE2 | iphc::DAM_MODE2 => {
            ip_addr.0[8..16].copy_from_slice(&iphc::MAC_BASE);
            ip_addr.0[14..16].copy_from_slice(&buf[*consumed..*consumed + 2]);
            *consumed += 2;
        }
        // SAM, DAM = 11: 0 bits
        // Suffix is the IID computed from the encapsulating header
        iphc::SAM_MODE3 | iphc::DAM_MODE3 => {
            let iid = compute_iid(mac_addr);
            ip_addr.0[8..16].copy_from_slice(&iid[0..8]);
        }
        _ => panic!("Unreachable case"),
    }
    // The bits covered by the provided context are always used, so we copy
    // the context bits into the address after the non-context bits are set.
    ip_addr.set_prefix(&ctx.prefix, ctx.prefix_len);
    Ok(())
}

// Returns the UDP ports in host byte-order
fn decompress_udp_ports(udp_nhc: u8, buf: &[u8], consumed: &mut usize) -> (u16, u16) {
    let src_compressed = (udp_nhc & nhc::UDP_SRC_PORT_FLAG) != 0;
    let dst_compressed = (udp_nhc & nhc::UDP_DST_PORT_FLAG) != 0;

    let src_port;
    let dst_port;
    if src_compressed && dst_compressed {
        // Both src and dst are compressed to 4 bits
        let src_short = ((buf[*consumed] >> 4) & 0xf) as u16;
        let dst_short = (buf[*consumed] & 0xf) as u16;
        src_port = nhc::UDP_4BIT_PORT | src_short;
        dst_port = nhc::UDP_4BIT_PORT | dst_short;
        *consumed += 1;
    } else if src_compressed {
        // Source port is compressed to 8 bits
        src_port = nhc::UDP_8BIT_PORT | (buf[*consumed] as u16);
        // Destination port is uncompressed
        dst_port = u16::from_be(slice_to_u16(&buf[*consumed + 1..*consumed + 3]));
        *consumed += 3;
    } else if dst_compressed {
        // Source port is uncompressed
        src_port = u16::from_be(slice_to_u16(&buf[*consumed..*consumed + 2]));
        // Destination port is compressed to 8 bits
        dst_port = nhc::UDP_8BIT_PORT | (buf[*consumed + 2] as u16);
        *consumed += 3;
    } else {
        // Both ports are uncompressed
        src_port = u16::from_be(slice_to_u16(&buf[*consumed..*consumed + 2]));
        dst_port = u16::from_be(slice_to_u16(&buf[*consumed + 2..*consumed + 4]));
        *consumed += 4;
    }
    (src_port, dst_port)
}

// Returns the UDP checksum in host byte-order
fn decompress_udp_checksum(
    udp_nhc: u8,
    udp_header: &[u8],
    udp_length: u16,
    ip6_header: &IP6Header,
    buf: &[u8],
    consumed: &mut usize,
) -> u16 {
    if (udp_nhc & nhc::UDP_CHECKSUM_FLAG) != 0 {
        // TODO: Need to verify that the packet was sent with *some* kind
        // of integrity check at a lower level (otherwise, we need to drop
        // the packet)
        compute_udp_checksum(ip6_header, udp_header, udp_length, &buf[*consumed..])
    } else {
        let checksum = u16::from_be(slice_to_u16(&buf[*consumed..*consumed + 2]));
        *consumed += 2;
        checksum
    }
}
