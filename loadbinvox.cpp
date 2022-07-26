/* This is a file copied from http://www.patrickmin.com/binvox/binvox.html
 * The program uses a third party tool 'binvox' to voxelize .obj files
 * credits: http://www.patrickmin.com
 */

#include "loadbinvox.h"

void LoadBinVox::getVoxels(byte *&voxObject)
{
    // Copy voxel values
    voxObject = voxels;
}

int LoadBinVox::getDims()
{
    return size;
}

bool LoadBinVox::loadBinVox(QString &filespec)
{

    ifstream *input = new ifstream(filespec.toStdString().c_str(), ios::in | ios::binary);

    //
    // read header
    //
    string line;
    *input >> line;  // #binvox
    if (line.compare("#binvox") != 0) {
      cout << "Error: first line reads [" << line << "] instead of [#binvox]" << endl;
      delete input;
      return 0;
    }
    int version;
    *input >> version;
    cout << "reading binvox version " << version << endl;

    int depth, height, width;
    depth = -1;
    int done = 0;
    while(input->good() && !done) {
      *input >> line;
      if (line.compare("data") == 0) done = 1;
      else if (line.compare("dim") == 0) {
        *input >> depth >> height >> width;
      }
      else {
        cout << "  unrecognized keyword [" << line << "], skipping" << endl;
        char c;
        do {  // skip until end of line
          c = input->get();
        } while(input->good() && (c != '\n'));

      }
    }
    if (!done) {
      cout << "  error reading header" << endl;
      return 0;
    }
    if (depth == -1) {
      cout << "  missing dimensions in header" << endl;
      return 0;
    }

    size = width * height * depth;
    voxels = new byte[size];
    if (!voxels) {
      cout << "  error allocating memory" << endl;
      return 0;
    }

    //
    // read voxel data
    //
    byte value;
    byte count;
    int index = 0;
    int end_index = 0;
    nr_voxels = 0;

    input->unsetf(ios::skipws);  // need to read every byte now (!)
    *input >> value;  // read the linefeed char

    while((end_index < size) && input->good()) {
      *input >> value >> count;

      if (input->good()) {
        end_index = index + count;
        if (end_index > size) return 0;
        for(int i=index; i < end_index; i++) voxels[i] = value;

        if (value) nr_voxels += count;
        index = end_index;
      }  // if file still ok

    }  // while

    input->close();
    cout << "  read " << nr_voxels << " voxels" << endl;

    return 1;
}
