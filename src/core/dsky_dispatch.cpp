#include "dsky_dispatch.h"

namespace dskymatic {

namespace {

// AGC channel 010 encodes each digit as a 5-bit pattern that isn't 0..9;
// this table matches apolloa's DecodeDigit.
int decode_digit(int enc) {
    switch (enc) {
        case 21: return 0;
        case  3: return 1;
        case 25: return 2;
        case 27: return 3;
        case 15: return 4;
        case 30: return 5;
        case 28: return 6;
        case 19: return 7;
        case 29: return 8;
        case 31: return 9;
        default: return -1;
    }
}

int combine_sign(bool plus, bool minus) {
    if (plus && !minus) return +1;
    if (minus && !plus) return -1;
    return 0;
}

} // namespace

DskyDispatch::DskyDispatch()
    : m_r1p(false), m_r1m(false),
      m_r2p(false), m_r2m(false),
      m_r3p(false), m_r3m(false)
{}

void DskyDispatch::reset_sign_state() {
    m_r1p = m_r1m = false;
    m_r2p = m_r2m = false;
    m_r3p = m_r3m = false;
}

void DskyDispatch::on_ch010(unsigned short w, IDskyDisplaySink &sink) {
    const int cmd  = (w >> 11) & 037;
    const int nw1  = (w >>  5) & 037;
    const int nw2  =  w        & 037;
    const bool flg = (w & (1 << 10)) != 0;

    switch (cmd) {
    case  1:  // R3- side
        m_r3m = flg;
        sink.set_sign("R3S", combine_sign(m_r3p, m_r3m));
        sink.set_digit("R3D4", decode_digit(nw1));
        sink.set_digit("R3D5", decode_digit(nw2));
        break;
    case  2:  // R3+ side
        m_r3p = flg;
        sink.set_sign("R3S", combine_sign(m_r3p, m_r3m));
        sink.set_digit("R3D2", decode_digit(nw1));
        sink.set_digit("R3D3", decode_digit(nw2));
        break;
    case  3:
        sink.set_digit("R2D5", decode_digit(nw1));
        sink.set_digit("R3D1", decode_digit(nw2));
        break;
    case  4:  // R2- side
        m_r2m = flg;
        sink.set_sign("R2S", combine_sign(m_r2p, m_r2m));
        sink.set_digit("R2D3", decode_digit(nw1));
        sink.set_digit("R2D4", decode_digit(nw2));
        break;
    case  5:  // R2+ side
        m_r2p = flg;
        sink.set_sign("R2S", combine_sign(m_r2p, m_r2m));
        sink.set_digit("R2D1", decode_digit(nw1));
        sink.set_digit("R2D2", decode_digit(nw2));
        break;
    case  6:  // R1- side
        m_r1m = flg;
        sink.set_sign("R1S", combine_sign(m_r1p, m_r1m));
        sink.set_digit("R1D4", decode_digit(nw1));
        sink.set_digit("R1D5", decode_digit(nw2));
        break;
    case  7:  // R1+ side
        m_r1p = flg;
        sink.set_sign("R1S", combine_sign(m_r1p, m_r1m));
        sink.set_digit("R1D2", decode_digit(nw1));
        sink.set_digit("R1D3", decode_digit(nw2));
        break;
    case  8:  // R1D1 only
        sink.set_digit("R1D1", decode_digit(nw2));
        break;
    case  9:  // Noun
        sink.set_digit("ND1", decode_digit(nw1));
        sink.set_digit("ND2", decode_digit(nw2));
        break;
    case 10:  // Verb
        sink.set_digit("VD1", decode_digit(nw1));
        sink.set_digit("VD2", decode_digit(nw2));
        break;
    case 11:  // Prog
        sink.set_digit("PD1", decode_digit(nw1));
        sink.set_digit("PD2", decode_digit(nw2));
        break;
    case 12:
        sink.set_light("NOATT_LIGHT",       (w & (1 <<  3)) != 0);
        sink.set_light("GIMBAL_LOCK_LIGHT", (w & (1 <<  5)) != 0);
        sink.set_light("TRACKER_LIGHT",     (w & (1 <<  7)) != 0);
        sink.set_light("PROG_LIGHT",        (w & (1 <<  8)) != 0);
        break;
    default:
        break;
    }
}

void DskyDispatch::on_ch011(unsigned short w, IDskyDisplaySink &sink) {
    // Bit numbering matches the Virtual AGC developer notes
    // (channel 011 is 1-indexed in the docs; LSB is bit 1).
    const bool comp_acty = (w & (1 << 1)) != 0;  // bit 2
    const bool uplink    = (w & (1 << 2)) != 0;  // bit 3
    const bool temp      = (w & (1 << 3)) != 0;  // bit 4
    const bool keyrel    = (w & (1 << 4)) != 0;  // bit 5
    const bool vnflash   = (w & (1 << 5)) != 0;  // bit 6
    const bool oprerr    = (w & (1 << 6)) != 0;  // bit 7

    sink.set_light("ELCOMPACTY",    comp_acty);
    sink.set_light("UPLINK_LIGHT",  uplink);
    sink.set_light("TEMP_LIGHT",    temp);
    sink.set_light("KEYREL_LIGHT",  keyrel);
    sink.set_light("OPR_ERR_LIGHT", oprerr);

    // V/N FLASH: both digit groups blink together at the shared
    // flasher rate.
    sink.set_flashing("VERB", vnflash);
    sink.set_flashing("NOUN", vnflash);

    // In the Block II DSKY the single hardware flasher also gates
    // the drive to the TEMP, KEY REL and OPR ERR latching relays,
    // so the lamp physically pulses in sync with V/N FLASH while
    // the AGC asserts the bit.  (See Virtual AGC "fictitious" ch0163,
    // which models exactly this post-flasher drive.)
    sink.set_flashing("TEMP_LIGHT",    temp);
    sink.set_flashing("KEYREL_LIGHT",  keyrel);
    sink.set_flashing("OPR_ERR_LIGHT", oprerr);
}

} // namespace dskymatic
