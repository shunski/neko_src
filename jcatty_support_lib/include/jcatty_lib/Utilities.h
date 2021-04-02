#ifndef UTILITIES_H
#define UTILITIES_H

#define default_queue_size 100;

typedef unsigned char Uint8;
typedef unsigned short Uint16;

typedef std::vector<Uint8> Uint8Sequence;
typedef std::vector<Uint16> Uint16Sequence;

enum PartID { HEAD, CHEST, WAIST, RFLEG, LFLEG, RHLEG, LHLEG };

#endif
