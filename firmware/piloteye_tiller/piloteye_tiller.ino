// PilotsEYE / Simplayer tiller - clean replacement firmware.
//
// All code lives in piloteye_tiller.cpp. It is kept as a .cpp (not inside this
// .ino) on purpose: the Arduino sketch preprocessor runs ctags on .ino files to
// auto-generate function prototypes, and the only ctags reachable on this
// machine (Universal Ctags) emits prototypes without return types, which breaks
// the build. A .cpp is compiled as plain C++ and skips that step entirely.
//
// This .ino is intentionally empty apart from this note.
