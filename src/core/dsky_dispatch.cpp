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
    sink.set_light("ELCOMPACTY",   (w & (1 << 1)) != 0);
    sink.set_light("UPLINK_LIGHT", (w & (1 << 2)) != 0);
    sink.set_light("TEMP_LIGHT",   (w & (1 << 3)) != 0);
    sink.set_light("KEYREL_LIGHT", (w & (1 << 4)) != 0);
    // BIT6 flashes the VERB/NOUN digit groups at ~1.5 cps.
    const bool flash = (w & (1 << 5)) != 0;
    sink.set_flashing("VERB", flash);
    sink.set_flashing("NOUN", flash);
    sink.set_light("OPR_ERR_LIGHT", (w & (1 << 6)) != 0);
}

} // namespace dskymatic
