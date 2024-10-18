#pragma once
#include "epd_driver.h"
const uint8_t digital7Bitmaps[2433] = {
    0x78, 0x9C, 0xED, 0xD6, 0xCD, 0x0D, 0x82, 0x40, 0x10, 0x05, 0xE0, 0x31, 0x51, 0x09, 0xE1, 0x02,
    0x1D, 0xD8, 0x91, 0x25, 0x98, 0x58, 0x86, 0x17, 0x28, 0x01, 0x2B, 0xC0, 0xB3, 0x27, 0x3B, 0x10,
    0x5B, 0xB1, 0x06, 0x0F, 0x20, 0x1B, 0xC7, 0xFD, 0x51, 0x51, 0xF7, 0x31, 0x5C, 0x35, 0xD9, 0x49,
    0x38, 0x7D, 0x79, 0x2C, 0xBB, 0x10, 0xF2, 0x88, 0xE8, 0xCC, 0x68, 0x8E, 0x64, 0x67, 0x71, 0x83,
    0xCA, 0x89, 0x55, 0x1C, 0x7D, 0x86, 0x4F, 0x03, 0x5A, 0x19, 0x5C, 0xA7, 0x0A, 0x6B, 0x5C, 0x98,
    0x55, 0x23, 0x1C, 0xAE, 0xA8, 0x4B, 0xF4, 0xAA, 0x39, 0x0E, 0xC7, 0x07, 0xBD, 0xF2, 0x95, 0x19,
    0x86, 0x75, 0x94, 0x79, 0xAE, 0x2F, 0x18, 0xD6, 0x51, 0xA7, 0x28, 0x6C, 0xA3, 0x4E, 0x41, 0xD8,
    0x46, 0x9D, 0x72, 0xB4, 0xDD, 0x7F, 0xCE, 0x86, 0x54, 0xAF, 0x39, 0x7D, 0x4F, 0xCD, 0xBD, 0xB2,
    0xA7, 0x97, 0xA0, 0x41, 0x83, 0x7A, 0xBA, 0xF2, 0x74, 0xF7, 0xA6, 0x33, 0x4F, 0xCD, 0x3F, 0xE1,
    0xA1, 0x4B, 0x1F, 0x5D, 0xD8, 0xEA, 0x14, 0xA8, 0x0D, 0x1B, 0x45, 0x51, 0x17, 0x36, 0x0A, 0xA3,
    0x44, 0xAD, 0xD6, 0x34, 0x9B, 0x60, 0xA4, 0x34, 0xCB, 0x68, 0x58, 0xCD, 0x0C, 0xDF, 0xB9, 0x91,
    0x9E, 0xAA, 0x10, 0x77, 0xD4, 0x48, 0xA7, 0x51, 0x8A, 0x27, 0xD9, 0x4A, 0x6F, 0xA1, 0xFC, 0x81,
    0x6F, 0x23, 0x68, 0xD0, 0xFF, 0x52, 0xB9, 0x15, 0x88, 0x8D, 0x02, 0xB5, 0x91, 0xFA, 0xA5, 0xB0,
    0xC9, 0x28, 0x1E, 0x6D, 0x41, 0x72, 0x83, 0x92, 0xDB, 0x97, 0xDC, 0xDC, 0xE4, 0xD6, 0x37, 0xD2,
    0x18, 0x47, 0xDA, 0xA6, 0xDC, 0x54, 0xA5, 0x96, 0x7B, 0x07, 0x83, 0x3B, 0x4D, 0x0B, 0x78, 0x9C,
    0xD5, 0xD2, 0xB1, 0x11, 0x80, 0x20, 0x0C, 0x05, 0xD0, 0x5F, 0xD9, 0xD8, 0xC8, 0xC6, 0x8C, 0xC0,
    0x08, 0x8C, 0xE0, 0x06, 0x32, 0x94, 0x05, 0x08, 0x77, 0x11, 0x7E, 0xE2, 0x69, 0x6F, 0xA3, 0x29,
    0x78, 0x17, 0x0E, 0x0A, 0xC8, 0x07, 0x00, 0x0F, 0x56, 0x9D, 0xC7, 0xBA, 0xCA, 0xC6, 0x46, 0x64,
    0x1A, 0x0D, 0xA9, 0x64, 0x34, 0x9D, 0x46, 0x92, 0x90, 0xFD, 0xCF, 0x44, 0x05, 0x87, 0x12, 0x15,
    0xB6, 0x9D, 0xA8, 0xA0, 0x28, 0x8B, 0x73, 0xB8, 0x2B, 0x73, 0xD3, 0xEB, 0x91, 0x4C, 0x82, 0x5E,
    0x2F, 0x24, 0x7C, 0xE1, 0x7D, 0x2F, 0xB1, 0x11, 0xDB, 0xC0, 0x91, 0xEC, 0x7B, 0x9A, 0x3C, 0xA3,
    0x61, 0x41, 0xB9, 0x62, 0x63, 0x21, 0x1A, 0x91, 0x3A, 0x01, 0x29, 0xA8, 0xB1, 0x9F, 0x78, 0x9C,
    0xED, 0xD6, 0xCF, 0x0D, 0x82, 0x30, 0x14, 0xC7, 0xF1, 0x67, 0xA2, 0xFC, 0x09, 0x17, 0xD8, 0x80,
    0x8D, 0x1C, 0xC1, 0xC4, 0x31, 0xBC, 0xE0, 0x02, 0xC6, 0x38, 0x01, 0x9C, 0x4D, 0x4C, 0xDC, 0xC0,
    0x3F, 0x7B, 0x78, 0x61, 0x06, 0x2F, 0x22, 0xA1, 0x96, 0xD2, 0xA6, 0x60, 0x7F, 0x6D, 0x18, 0xA0,
    0xEF, 0xFA, 0xC9, 0x17, 0x12, 0x48, 0x5E, 0x1E, 0x11, 0xD5, 0x0C, 0xCD, 0x8D, 0xC4, 0xE4, 0x1D,
    0x54, 0x96, 0x08, 0xC5, 0xA9, 0x8A, 0x1F, 0x16, 0x2D, 0x85, 0xA6, 0x2D, 0xD6, 0x78, 0xEF, 0x88,
    0x4B, 0xFA, 0x26, 0xF6, 0x38, 0xBE, 0x3A, 0xDE, 0xCC, 0x53, 0xC6, 0x02, 0x5B, 0xCC, 0x53, 0xA9,
    0x20, 0x16, 0xA9, 0x54, 0x33, 0x16, 0xA9, 0x54, 0x3A, 0x9D, 0xA7, 0xB3, 0xA3, 0x76, 0xA4, 0xC6,
    0xDC, 0x99, 0x4B, 0xDF, 0x5E, 0xBD, 0x7A, 0x9D, 0xA1, 0xD5, 0x58, 0x9F, 0xAF, 0xE9, 0x1C, 0xA8,
    0xD1, 0x6A, 0x2E, 0xB4, 0xA8, 0xD2, 0x6A, 0x2E, 0xB4, 0xA3, 0x88, 0x03, 0x9C, 0xCA, 0x38, 0xC0,
    0x69, 0x1F, 0x7F, 0x06, 0xC5, 0x6B, 0x34, 0x4A, 0xB3, 0x8C, 0xD2, 0x6C, 0x81, 0x55, 0x2C, 0x3A,
    0xBE, 0x78, 0x96, 0xE8, 0xC9, 0xC3, 0x9E, 0xE3, 0xBA, 0x46, 0x71, 0xA2, 0x14, 0xC5, 0x72, 0xB3,
    0xF7, 0x0A, 0xE2, 0x44, 0x2B, 0x5B, 0xFD, 0x7F, 0xC9, 0x0B, 0x8D, 0x74, 0x63, 0xF9, 0x17, 0x83,
    0x32, 0xAF, 0x5E, 0xBD, 0xCE, 0xD0, 0xC2, 0xA9, 0xA1, 0x79, 0x51, 0x68, 0x2D, 0xC0, 0x35, 0xA2,
    0x35, 0x44, 0x97, 0x8C, 0x52, 0x90, 0xAA, 0xB8, 0x81, 0xA9, 0x8A, 0x6B, 0x9C, 0xCA, 0x38, 0xEF,
    0x60, 0xAA, 0xE2, 0xAD, 0xF5, 0xEA, 0x13, 0xEC, 0xBE, 0x18, 0xDD, 0xD7, 0xA6, 0xFB, 0x52, 0x75,
    0x5D, 0xB9, 0x3F, 0x3E, 0xC9, 0x7A, 0x32, 0x78, 0x9C, 0xED, 0x96, 0x4D, 0x0E, 0x82, 0x30, 0x10,
    0x46, 0x67, 0xD5, 0x40, 0xBA, 0x81, 0x1B, 0x78, 0x4E, 0x57, 0x5C, 0xC0, 0xA0, 0x27, 0x80, 0xB5,
    0x89, 0x89, 0x37, 0x00, 0xBD, 0x87, 0x1B, 0xCF, 0xE0, 0xC2, 0x9F, 0xC6, 0xB1, 0x0C, 0xA9, 0x4A,
    0x3A, 0x33, 0x2E, 0x35, 0xB1, 0xB3, 0x7D, 0x79, 0xC0, 0x94, 0xE4, 0xEB, 0x07, 0x47, 0xE4, 0xA6,
    0x03, 0x3F, 0xB3, 0x3B, 0xCB, 0xD0, 0x7A, 0xC6, 0x6B, 0xA3, 0xB8, 0x13, 0x58, 0xE3, 0x59, 0xE1,
    0x78, 0x96, 0x57, 0xA2, 0xD8, 0xC0, 0xCD, 0x4A, 0x62, 0xBE, 0x15, 0xDF, 0xE8, 0x35, 0x44, 0xC3,
    0x8B, 0x5E, 0x23, 0xC6, 0x88, 0xA4, 0x11, 0x8B, 0x45, 0xD2, 0x88, 0xC1, 0x6A, 0x3D, 0x9D, 0x39,
    0xB8, 0x27, 0x8B, 0xA6, 0x47, 0x99, 0x9D, 0x12, 0x4B, 0xEC, 0xCF, 0x58, 0xFB, 0x62, 0xFB, 0xC3,
    0x74, 0x16, 0x70, 0x0D, 0x2C, 0x8E, 0x9F, 0xAC, 0x0D, 0x2C, 0x8E, 0x9F, 0x9A, 0x44, 0xC3, 0xA7,
    0x16, 0x89, 0x86, 0x4F, 0xAD, 0x1A, 0x2E, 0x03, 0xE3, 0xC3, 0x2E, 0x2B, 0xCA, 0x52, 0x0A, 0x42,
    0x3B, 0x2E, 0xC3, 0x3D, 0xB3, 0x83, 0xB3, 0xF8, 0x2D, 0xB6, 0x12, 0x77, 0x20, 0x4D, 0xD8, 0xDD,
    0x2E, 0x11, 0x85, 0x33, 0xDB, 0x0C, 0x1B, 0x48, 0x67, 0x4D, 0xDA, 0x8F, 0xFC, 0xF7, 0xC4, 0x12,
    0xFB, 0x36, 0x7B, 0xBB, 0x8B, 0x95, 0x3B, 0x9C, 0xB9, 0xFB, 0xFB, 0xC0, 0xB8, 0xCE, 0xE0, 0xF0,
    0x63, 0xD7, 0x50, 0x3A, 0x8A, 0xD6, 0x6D, 0x94, 0x4E, 0xA4, 0x75, 0x29, 0xAD, 0x83, 0x69, 0xDD,
    0x4D, 0xEB, 0x7C, 0x82, 0x48, 0x5D, 0xF1, 0x01, 0x4A, 0x0E, 0x7B, 0x91, 0x78, 0x9C, 0xED, 0xD6,
    0xDD, 0x0D, 0x82, 0x30, 0x10, 0xC0, 0xF1, 0x33, 0xF1, 0x2B, 0xE9, 0x0B, 0x6C, 0xC0, 0x46, 0x8E,
    0x60, 0xE2, 0x22, 0xB2, 0x80, 0xD1, 0x11, 0x58, 0xC0, 0x84, 0x0D, 0x20, 0xEE, 0xE1, 0x0B, 0x33,
    0xF0, 0xA0, 0xD6, 0x70, 0xF6, 0x4A, 0x01, 0xA5, 0x77, 0x8D, 0x03, 0xF4, 0x12, 0x9E, 0x7E, 0xF9,
    0x37, 0x94, 0x07, 0x72, 0x70, 0x00, 0x69, 0x72, 0x80, 0xAC, 0xDB, 0x48, 0xAA, 0x15, 0x34, 0x78,
    0x14, 0xB0, 0xC4, 0x0A, 0x5E, 0x88, 0x42, 0xAC, 0x11, 0xD7, 0xE6, 0xE1, 0xE3, 0x12, 0x7B, 0xE5,
    0x63, 0x3D, 0x28, 0x17, 0x53, 0xDA, 0x2B, 0x17, 0xBF, 0x27, 0xF5, 0xE3, 0x1A, 0x27, 0x45, 0x4F,
    0xDB, 0xA8, 0x51, 0xA3, 0x7A, 0xBA, 0xF7, 0xB4, 0xF8, 0xD2, 0xD5, 0xED, 0xFE, 0x3B, 0x27, 0xFA,
    0x27, 0x38, 0xDD, 0x65, 0x1D, 0xCE, 0x66, 0x5B, 0x8C, 0xBA, 0x6C, 0xE6, 0x88, 0x67, 0x1B, 0x93,
    0x32, 0xA9, 0x8B, 0x49, 0x99, 0x94, 0xE2, 0xA7, 0xD1, 0x24, 0x5D, 0x70, 0xA9, 0x89, 0x93, 0x34,
    0xA5, 0xB7, 0xE7, 0x55, 0x0D, 0x97, 0xE3, 0x4E, 0xAE, 0xE0, 0x61, 0x4E, 0x96, 0x62, 0x95, 0xA3,
    0x53, 0x26, 0xB6, 0xA9, 0x53, 0x3F, 0x56, 0x17, 0x1C, 0x15, 0xE6, 0x5F, 0xF2, 0x4A, 0xF7, 0x19,
    0xD5, 0x1B, 0x9B, 0x8A, 0xDA, 0x46, 0x8D, 0x1A, 0xF5, 0x0F, 0xAD, 0x83, 0xEA, 0x36, 0x0A, 0x49,
    0xEB, 0xA0, 0xDA, 0x58, 0xD6, 0x32, 0xA8, 0x76, 0x83, 0x92, 0x95, 0xB6, 0xAF, 0xC0, 0x68, 0x15,
    0xD2, 0x1C, 0xE0, 0x03, 0x2A, 0x5E, 0x65, 0xF4, 0x78, 0x9C, 0xED, 0xD6, 0xCB, 0x0D, 0x82, 0x40,
    0x14, 0x85, 0xE1, 0x6B, 0xA2, 0x22, 0x99, 0x0D, 0x76, 0x60, 0x47, 0x96, 0x60, 0x62, 0x19, 0x6E,
    0xA0, 0x01, 0xA3, 0x56, 0xC0, 0xDE, 0xC4, 0x84, 0x0E, 0x40, 0xFB, 0x70, 0x63, 0x0D, 0x2E, 0x54,
    0x88, 0x57, 0x19, 0x18, 0x1E, 0xCE, 0xB9, 0xC4, 0x02, 0x66, 0xB6, 0x5F, 0x7E, 0x26, 0xBA, 0x98,
    0x1C, 0x22, 0xBA, 0x31, 0x3A, 0x29, 0xE9, 0xB3, 0x78, 0x43, 0x65, 0xA5, 0x15, 0xA7, 0x26, 0x3E,
    0x0B, 0x1A, 0x97, 0xB8, 0x0E, 0x0A, 0xAC, 0xBE, 0xBE, 0xD5, 0xC3, 0x71, 0x5C, 0xDD, 0x1A, 0xE2,
    0xB8, 0x4C, 0xE9, 0xC5, 0x0C, 0x63, 0x9D, 0x52, 0xCE, 0x38, 0xF6, 0x8D, 0xA2, 0xB8, 0x4A, 0xB5,
    0x82, 0xD8, 0x6F, 0x95, 0xBD, 0xC3, 0xB1, 0x7F, 0x36, 0xD4, 0xD1, 0x90, 0x84, 0xA3, 0x95, 0x9D,
    0x3A, 0x75, 0xFA, 0x87, 0xAE, 0x06, 0x75, 0x72, 0xB9, 0xF6, 0xCF, 0xB6, 0xA3, 0x4B, 0xFB, 0x41,
    0x9B, 0xB5, 0x3A, 0xB6, 0x1F, 0xB4, 0x5D, 0xA3, 0x20, 0x35, 0x71, 0x0E, 0x53, 0x13, 0x07, 0xF3,
    0x11, 0x7E, 0x46, 0xEB, 0x9B, 0x85, 0x47, 0x56, 0x99, 0x5F, 0x85, 0xBE, 0x9C, 0xD2, 0x83, 0x79,
    0x2A, 0xC5, 0x2A, 0xE2, 0x5A, 0x41, 0xAC, 0xD3, 0x5A, 0xED, 0x58, 0xED, 0xB9, 0x51, 0xFA, 0xFD,
    0x27, 0x4F, 0xF4, 0xEC, 0xA8, 0x75, 0x74, 0x2A, 0xEA, 0xDD, 0xA9, 0x53, 0xA7, 0x7F, 0x68, 0xD6,
    0x55, 0x7B, 0x51, 0x14, 0xAD, 0x82, 0x35, 0x92, 0xB5, 0x8A, 0x96, 0x4C, 0x61, 0x14, 0xAE, 0xA0,
    0xC4, 0x28, 0x5E, 0x50, 0x79, 0xA5, 0xC2, 0xFA, 0x4A, 0xAA, 0xDD, 0x27, 0x2D, 0xB7, 0x5C, 0x89,
    0xE9, 0x37, 0x8E, 0xE4, 0xD4, 0x0C, 0xB0, 0xE1, 0xB5, 0x39, 0xBC, 0x54, 0x87, 0x56, 0xEE, 0x07,
    0x82, 0x61, 0x79, 0xCD, 0x78, 0x9C, 0xED, 0xD6, 0xCD, 0x0D, 0x82, 0x30, 0x18, 0xC6, 0xF1, 0xD7,
    0x44, 0x41, 0xD2, 0x0B, 0x6E, 0xE0, 0x46, 0x8E, 0x60, 0xE2, 0x18, 0x5E, 0x60, 0x01, 0xA3, 0x4E,
    0xC0, 0xDD, 0xC4, 0x84, 0x0D, 0x40, 0xF7, 0xF0, 0xE2, 0x0C, 0x1E, 0x54, 0x88, 0xAF, 0x50, 0x28,
    0x1F, 0xE9, 0xD3, 0xEA, 0x00, 0xED, 0xF5, 0x97, 0xBF, 0xA8, 0x24, 0xCD, 0x43, 0x44, 0x77, 0x46,
    0x27, 0x23, 0x79, 0x96, 0x1F, 0xA8, 0x2C, 0xA4, 0xE2, 0x54, 0xC5, 0x17, 0x83, 0x26, 0x35, 0x6E,
    0xC2, 0x12, 0x6B, 0x20, 0x9F, 0xEA, 0xE3, 0x38, 0x69, 0x9E, 0x1A, 0xE1, 0xB8, 0x4E, 0xE9, 0xCD,
    0x0C, 0x63, 0x99, 0x52, 0xC1, 0x38, 0x0E, 0x94, 0xA2, 0xB8, 0x49, 0xA5, 0x82, 0x38, 0xE8, 0x95,
    0xFD, 0xE3, 0x69, 0x7C, 0xB6, 0x34, 0xD0, 0x88, 0x0C, 0x47, 0x2A, 0x3B, 0x75, 0xEA, 0xF4, 0x0F,
    0x5D, 0x5B, 0x75, 0x76, 0xBD, 0x8D, 0xCF, 0x6E, 0xA0, 0x2B, 0xFD, 0x42, 0x9B, 0xF7, 0x3A, 0xD5,
    0x2F, 0xB4, 0x7D, 0xA7, 0x20, 0x55, 0x71, 0x01, 0x53, 0x15, 0x87, 0x8B, 0x09, 0xBE, 0x46, 0xE7,
    0x56, 0x15, 0xB6, 0x4F, 0xCE, 0xE8, 0xC9, 0xEC, 0x99, 0xBE, 0x95, 0x88, 0xB9, 0x51, 0x14, 0xCB,
    0xB4, 0x51, 0x10, 0x8B, 0x03, 0x2B, 0xD5, 0xFF, 0xC9, 0x33, 0xBD, 0x7A, 0xD5, 0xDF, 0x82, 0x4C,
    0x5B, 0xD5, 0xDF, 0xE0, 0xC3, 0xA9, 0x53, 0xA7, 0x9A, 0xEA, 0xAB, 0x20, 0x1F, 0x28, 0x58, 0x14,
    0x65, 0xA7, 0x68, 0x8D, 0xE4, 0x9D, 0xC2, 0x25, 0x53, 0xB6, 0x8A, 0x57, 0x50, 0x5A, 0xAB, 0x79,
    0x41, 0x55, 0x9D, 0x67, 0x5E, 0x5F, 0x69, 0x75, 0xB5, 0x58, 0x96, 0x5B, 0x21, 0x6C, 0xAB, 0x2F,
    0xAE, 0x7F, 0xB8, 0x75, 0x31, 0xFE, 0x58, 0x9B, 0xF6, 0xA5, 0x6A, 0x5B, 0xB9, 0x5F, 0x9C, 0x9A,
    0x51, 0x12, 0x78, 0x9C, 0xED, 0xD6, 0xC1, 0x0D, 0x82, 0x50, 0x0C, 0x80, 0xE1, 0x9A, 0xA8, 0x84,
    0x70, 0x81, 0x0D, 0xDC, 0xC8, 0x11, 0x4C, 0x1C, 0xC3, 0x0B, 0x8C, 0x80, 0x13, 0xE0, 0xD9, 0x13,
    0x1B, 0x88, 0xAB, 0x38, 0x03, 0x07, 0x50, 0x62, 0x7D, 0xAF, 0x0F, 0x82, 0xFA, 0xDA, 0xEA, 0x00,
    0xAF, 0x09, 0xA7, 0x2F, 0xFF, 0xEB, 0x8D, 0x14, 0x00, 0x6E, 0xC8, 0xCD, 0x05, 0x68, 0x36, 0x4F,
    0x56, 0x31, 0x21, 0xE5, 0xD3, 0x29, 0xBE, 0x0A, 0x5A, 0x59, 0xDC, 0xA7, 0x03, 0xAF, 0x71, 0x61,
    0xB7, 0x46, 0x7C, 0x5C, 0xC1, 0x23, 0x31, 0x5B, 0x73, 0x3E, 0x8E, 0x6B, 0xB3, 0xF9, 0x8E, 0xC8,
    0xC6, 0x26, 0x45, 0x5C, 0x9B, 0x8F, 0x8D, 0x4D, 0xEA, 0x94, 0x8B, 0x29, 0x75, 0xCA, 0xC4, 0x94,
    0x3A, 0xC5, 0xE8, 0x78, 0xFE, 0x9C, 0x03, 0x0C, 0xB3, 0xE6, 0xF0, 0x3D, 0x0D, 0xCE, 0x8A, 0x9E,
    0xB6, 0x41, 0x83, 0x06, 0xF5, 0x74, 0xE7, 0xE9, 0xE9, 0x4D, 0x57, 0x9E, 0xDA, 0x7F, 0xC2, 0xA8,
    0x5B, 0x1F, 0x5D, 0x4C, 0xBA, 0x64, 0x94, 0x62, 0xAB, 0x5C, 0xEA, 0x62, 0xAB, 0x6C, 0x0A, 0xD0,
    0x1B, 0x4D, 0xB3, 0x05, 0x8F, 0x90, 0x66, 0x99, 0x20, 0xBF, 0xA7, 0x33, 0x2F, 0x8B, 0x58, 0xA0,
    0xA6, 0x9D, 0xA6, 0x25, 0x6A, 0xDA, 0x6B, 0x5A, 0xA2, 0xA6, 0x6D, 0xD0, 0xA0, 0x41, 0xFF, 0xD0,
    0x46, 0xD5, 0xF1, 0xA2, 0x90, 0xB4, 0x51, 0x95, 0x62, 0x59, 0x6B, 0x55, 0xE9, 0x82, 0x92, 0xB5,
    0x9E, 0x8E, 0x46, 0x21, 0x4E, 0x34, 0x2D, 0x00, 0x5E, 0xB6, 0x43, 0x7F, 0x55, 0x78, 0x9C, 0xED,
    0x96, 0xCB, 0x0D, 0x82, 0x40, 0x10, 0x40, 0xC7, 0x44, 0x05, 0xB2, 0x17, 0xE8, 0xC0, 0x8E, 0x2C,
    0xC1, 0xC4, 0x32, 0xBC, 0x40, 0x03, 0x46, 0xAD, 0x00, 0xCE, 0x26, 0x26, 0x76, 0x00, 0xDA, 0x87,
    0x17, 0x6B, 0xF0, 0xE0, 0x87, 0x38, 0xEE, 0x2E, 0x08, 0xE2, 0xCE, 0x0E, 0x57, 0x0E, 0x3B, 0x89,
    0xA7, 0x97, 0xC7, 0xCA, 0x27, 0x93, 0x07, 0x00, 0x57, 0xA4, 0x26, 0x07, 0x3D, 0xB3, 0x37, 0x49,
    0x51, 0x68, 0x4A, 0xAB, 0x5F, 0xF9, 0x64, 0xA1, 0xA9, 0x82, 0xCB, 0xB0, 0xA4, 0x69, 0x90, 0xA8,
    0x53, 0x3D, 0x5A, 0x4E, 0xE1, 0x25, 0xE4, 0xA9, 0x31, 0x2D, 0x07, 0x47, 0x79, 0xF2, 0x13, 0x91,
    0x94, 0xA5, 0x8A, 0x38, 0x95, 0x3F, 0x52, 0x96, 0x6A, 0x45, 0x29, 0x59, 0xAB, 0x15, 0x25, 0x64,
    0xAD, 0x56, 0x14, 0xBD, 0xDD, 0xBE, 0x3B, 0x2B, 0x28, 0x5B, 0x1A, 0xC3, 0xFF, 0x14, 0xD8, 0x52,
    0x34, 0xE8, 0xCD, 0x51, 0x47, 0x1D, 0x35, 0xE8, 0xC2, 0xA0, 0xD9, 0x0F, 0x9D, 0x9C, 0x2F, 0xDD,
    0x59, 0xAB, 0x9D, 0x50, 0xD3, 0xB9, 0xB9, 0xD0, 0xFC, 0xAC, 0xA1, 0x63, 0x73, 0xA1, 0x6D, 0xB4,
    0xAC, 0x28, 0xA1, 0xD6, 0xB2, 0xA2, 0x84, 0xAA, 0xE4, 0x87, 0xA4, 0x61, 0x34, 0xA2, 0xD7, 0xA8,
    0x1F, 0x46, 0x11, 0x58, 0xA9, 0x5E, 0xB1, 0xB6, 0x2B, 0xE7, 0x70, 0x67, 0xFE, 0x95, 0x48, 0x98,
    0x3B, 0xD2, 0xAA, 0xF5, 0x69, 0x88, 0x2D, 0xA2, 0xF5, 0x49, 0x1E, 0xD4, 0xFD, 0xD8, 0xDF, 0x82,
    0x56, 0x07, 0xFD, 0xE5, 0x38, 0xEA, 0xE8, 0xD0, 0x28, 0x5F, 0x05, 0x6C, 0x51, 0x50, 0x35, 0x52,
    0x34, 0x94, 0x2C, 0x99, 0x12, 0x7B, 0x2B, 0x88, 0x2F, 0x28, 0xBE, 0xBE, 0xF8, 0x72, 0xE3, 0xAB,
    0xAF, 0xA7, 0x18, 0x7B, 0x6A, 0x93, 0x2F, 0x55, 0xAE, 0x72, 0x3F, 0xC3, 0xDB, 0x2A, 0x1B, 0x78,
    0x9C, 0xED, 0xD6, 0xCB, 0x0D, 0x82, 0x40, 0x10, 0xC6, 0xF1, 0x31, 0xF1, 0x99, 0xBD, 0x40, 0x07,
    0x76, 0x64, 0x09, 0x26, 0x96, 0xE1, 0x45, 0x1A, 0x30, 0x6A, 0x05, 0x78, 0x36, 0x31, 0xB1, 0x03,
    0xD1, 0x3E, 0xBC, 0x58, 0x03, 0x07, 0x1F, 0xC4, 0x71, 0x5F, 0x80, 0xB8, 0xDF, 0xA2, 0x05, 0xEC,
    0x24, 0x9E, 0x7E, 0xF9, 0xB3, 0xC2, 0x61, 0x33, 0x44, 0x74, 0x65, 0x34, 0x47, 0xD2, 0x33, 0x7E,
    0x41, 0x65, 0xA1, 0x15, 0xA7, 0x65, 0x7C, 0xF2, 0x68, 0xAA, 0x70, 0x16, 0x15, 0x58, 0x47, 0x89,
    0x3A, 0x75, 0x80, 0xE3, 0x94, 0x9E, 0x42, 0x9E, 0xBA, 0xC0, 0xF1, 0xE8, 0x20, 0x4F, 0x7E, 0x30,
    0xC3, 0x58, 0xA6, 0xCC, 0x7D, 0xF9, 0x83, 0xB1, 0x4C, 0x8D, 0xA2, 0x58, 0xA7, 0x46, 0x41, 0xAC,
    0x53, 0xA3, 0x3C, 0xD8, 0xEC, 0x9A, 0x33, 0xA7, 0xA2, 0xD6, 0x05, 0x7D, 0x4F, 0xC6, 0xB5, 0xB2,
    0xA3, 0x79, 0xD0, 0xA0, 0x41, 0x1D, 0x9D, 0x3A, 0xBA, 0xFD, 0xD0, 0xDE, 0xF9, 0xD2, 0x9C, 0xA5,
    0xBA, 0x13, 0xAC, 0x4E, 0xDC, 0x0B, 0x6D, 0xB8, 0xAD, 0xB4, 0xEB, 0x5E, 0x68, 0x2B, 0x1D, 0x2B,
    0x05, 0xA9, 0x8D, 0x95, 0x82, 0x54, 0xC5, 0x77, 0xA9, 0x51, 0xDC, 0xC1, 0xD7, 0xE8, 0x30, 0x8A,
    0x63, 0xF5, 0xEF, 0xB1, 0x8A, 0xF2, 0xE5, 0xD0, 0x93, 0x8F, 0x74, 0x93, 0x4F, 0xF6, 0xC5, 0x22,
    0x61, 0xAB, 0x20, 0xD6, 0xA9, 0x55, 0x37, 0x16, 0x6B, 0xAE, 0x94, 0xBE, 0xBF, 0xE4, 0x5E, 0xBD,
    0x4F, 0xA5, 0xCE, 0xE8, 0xD4, 0xAB, 0x79, 0xD0, 0xA0, 0x41, 0xFF, 0xD0, 0xEC, 0x53, 0x7D, 0x1B,
    0x85, 0x46, 0xB0, 0x8D, 0x64, 0xB5, 0xA2, 0x4D, 0xA6, 0x28, 0xD5, 0xBF, 0x05, 0xE1, 0xB4, 0xDC,
    0xA0, 0x3C, 0xA9, 0xDD, 0xBE, 0x3C, 0xA9, 0xDD, 0xDC, 0x7C, 0xA9, 0xD9, 0xFA, 0x7E, 0x6C, 0x8C,
    0x3F, 0xB6, 0xCD, 0xF6, 0x4D, 0xB5, 0x6D, 0xCB, 0x7D, 0x03, 0xA9, 0xA2, 0x52, 0xD6, 0x78, 0x9C,
    0x63, 0x50, 0x38, 0x79, 0x56, 0x82, 0x81, 0x81, 0xE1, 0xEB, 0xFF, 0xFF, 0xEF, 0x99, 0x19, 0x1C,
    0xFE, 0x03, 0x81, 0x3E, 0xC3, 0x05, 0x10, 0xB5, 0x9E, 0xE1, 0x33, 0x88, 0xFA, 0xCF, 0xF0, 0x0D,
    0x4C, 0x31, 0x7F, 0x05, 0x53, 0x4C, 0x9F, 0x40, 0xE4, 0x7B, 0x86, 0x0D, 0x20, 0xAA, 0x9F, 0x41,
    0x01, 0x44, 0xC9, 0x32, 0x30, 0x5C, 0xFE, 0xFF, 0xFF, 0x3C, 0x23, 0xD0, 0x98, 0xA9, 0x5D, 0x2C,
    0x0C, 0x43, 0x16, 0x08, 0x80, 0x7C, 0xC0, 0x90, 0xF0, 0xF7, 0xBF, 0x0F, 0x90, 0xFA, 0x05, 0xF4,
    0x18, 0x3B, 0x43, 0x01, 0xC8, 0x7F, 0xFE, 0x0C, 0x1F, 0x40, 0xD4, 0x79, 0x86, 0x2F, 0xE0, 0x20,
    0x60, 0x44, 0x0D, 0x90, 0x8F, 0x20, 0xF2, 0x3E, 0xC3, 0x04, 0x10, 0x95, 0xCF, 0xC0, 0xF0, 0x17,
    0x48, 0x71, 0x31, 0x80, 0xB8, 0xF5, 0x20, 0xD3, 0x1C, 0xDD, 0x80, 0x04, 0x00, 0xC9, 0xD1, 0x6A,
    0xE5,
};
const GFXglyph digital7Glyphs[] = {
    { 59, 94, 77, 9, 94, 238, 0 }, // 0
    { 13, 86, 77, 55, 90, 112, 238 }, // 1
    { 59, 94, 77, 9, 94, 249, 350 }, // 2
    { 55, 94, 77, 13, 94, 229, 599 }, // 3
    { 59, 86, 77, 9, 90, 220, 828 }, // 4
    { 59, 94, 77, 9, 94, 252, 1048 }, // 5
    { 59, 94, 77, 9, 94, 254, 1300 }, // 6
    { 59, 90, 77, 9, 94, 219, 1554 }, // 7
    { 59, 94, 77, 9, 94, 258, 1773 }, // 8
    { 59, 94, 77, 9, 94, 271, 2031 }, // 9
    { 13, 51, 19, 3, 71, 131, 2302 }, // :
};
const UnicodeInterval digital7Intervals[] = {
    { 0x30, 0x3A, 0x0 },
};
const GFXfont digital7 = {
    (uint8_t*)digital7Bitmaps,
    (GFXglyph*)digital7Glyphs,
    (UnicodeInterval*)digital7Intervals,
    1,
    1,
    102,
    94,
    0,
};
