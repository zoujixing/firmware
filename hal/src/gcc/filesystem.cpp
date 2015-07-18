#include "filesystem.h"
#include <stddef.h>
#include <string.h>
#include <cstdio>
#include <stdexcept>

const char* rootDir = NULL;

using namespace std;

void set_root_dir(const char* dir) {
    rootDir = dir;
}

void read_file(const char* filename, void* data, size_t length)
{
    char buf[256];
    buf[0] = 0;
    if (rootDir) {
        strcpy(buf, rootDir);
        strcat(buf, "/");
    }
    strcat(buf, filename);
    FILE *f = fopen(buf, "rb");
    if (f!=NULL) {
        fread(data, length, 1, f);
        fclose(f);
    }
    else
    {
        throw invalid_argument(string("unable to read file '") + buf + "'");
    }
}


