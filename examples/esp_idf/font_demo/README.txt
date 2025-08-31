//
// This demo shows how to use both virtual display surfaces and
// custom fonts. The fonts are created with the fontconvert tool (in this repo)
// and a TTF (TrueType font) file as the source.
// Use fontconvert to create the sizes and character ranges of fonts needed
// for your projects. Characters 32 to 127 are the standard ASCII set and
// 128 to 255 are the extended ASCII set which follows Microsoft's codepage 1252
// CP1252 consists of most symbols and accented characters from European
// Latin languages
// The strings passed to FastEPD are in ASCII or UTF-8 format
//
// The fontconvert tool example:
// ./fontconvert <source.ttf> <output.h> size start end
// For example, to create the 80pt Roboto-Black font used in this example, the
// command line arguments would be:
// ./fontconvert Roboto-Black.ttf Roboto_Black_80.h 80 32 255
// This creates the full ASCII and extended ASCII set of characters
//

