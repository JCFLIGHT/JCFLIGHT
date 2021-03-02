#include "SD.h"

namespace SDLib
{

#define MAX_COMPONENT_LEN 12
#define PATH_COMPONENT_BUFFER_LEN MAX_COMPONENT_LEN + 1

  bool getNextPathComponent(const char *path, unsigned int *p_offset, char *buffer)
  {
    int bufferOffset = 0;
    int offset = *p_offset;
    if (path[offset] == '/')
      offset++;
    while (bufferOffset < MAX_COMPONENT_LEN && (path[offset] != '/') && (path[offset] != '\0'))
      buffer[bufferOffset++] = path[offset++];
    buffer[bufferOffset] = '\0';
    if (path[offset] == '/')
      offset++;
    *p_offset = offset;
    return (path[offset] != '\0');
  }

  bool walkPath(const char *filepath, SdFile &parentDir, bool (*callback)(SdFile &parentDir, const char *filePathComponent, bool isLastComponent, void *object), void *object = NULL)
  {
    SdFile subfile1;
    SdFile subfile2;
    char buffer[PATH_COMPONENT_BUFFER_LEN];
    unsigned int offset = 0;
    SdFile *p_parent;
    SdFile *p_child;
    SdFile *p_tmp_sdfile;
    p_child = &subfile1;
    p_parent = &parentDir;
    while (true)
    {
      bool moreComponents = getNextPathComponent(filepath, &offset, buffer);
      bool shouldContinue = callback((*p_parent), buffer, !moreComponents, object);
      if (!shouldContinue)
      {
        if (p_parent != &parentDir)
          (*p_parent).close();
        return false;
      }
      if (!moreComponents)
        break;
      bool exists = (*p_child).open(*p_parent, buffer, O_RDONLY);
      if (p_parent != &parentDir)
        (*p_parent).close();
      if (exists)
      {
        if (p_parent == &parentDir)
          p_parent = &subfile2;
        p_tmp_sdfile = p_parent;
        p_parent = p_child;
        p_child = p_tmp_sdfile;
      }
      else
        return false;
    }
    if (p_parent != &parentDir)
      (*p_parent).close();
    return true;
  }

  bool callback_pathExists(SdFile &parentDir, const char *filePathComponent, bool, void *)
  {
    SdFile child;
    bool exists = child.open(parentDir, filePathComponent, O_RDONLY);
    if (exists)
      child.close();
    return exists;
  }

  bool callback_makeDirPath(SdFile &parentDir, const char *filePathComponent, bool isLastComponent, void *object)
  {
    bool result = false;
    SdFile child;
    result = callback_pathExists(parentDir, filePathComponent, isLastComponent, object);
    if (!result)
      result = child.makeDir(parentDir, filePathComponent);
    return result;
  }

  bool callback_remove(SdFile &parentDir, const char *filePathComponent, bool isLastComponent, void *)
  {
    if (isLastComponent)
      return SdFile::remove(parentDir, filePathComponent);
    return true;
  }

  bool callback_rmdir(SdFile &parentDir, const char *filePathComponent, bool isLastComponent, void *)
  {
    if (isLastComponent)
    {
      SdFile f;
      if (!f.open(parentDir, filePathComponent, O_READ))
        return false;
      return f.rmDir();
    }
    return true;
  }

  bool SDClass::begin(uint8_t csPin)
  {
    if (root.isOpen())
      root.close();
    return card.init(SPI_HALF_SPEED, csPin) && volume.init(card) && root.openRoot(volume);
  }

  bool SDClass::begin(uint32_t clock, uint8_t csPin)
  {
    if (root.isOpen())
      root.close();
    return card.init(SPI_HALF_SPEED, csPin) && card.setSpiClock(clock) && volume.init(card) && root.openRoot(volume);
  }

  void SDClass::end()
  {
    root.close();
  }

  SdFile SDClass::getParentDir(const char *filepath, int *index)
  {
    SdFile d1;
    SdFile d2;
    d1.openRoot(volume);
    SdFile *parent = &d1;
    SdFile *subdir = &d2;
    const char *origpath = filepath;
    while (strchr(filepath, '/'))
    {
      if (filepath[0] == '/')
      {
        filepath++;
        continue;
      }
      if (!strchr(filepath, '/'))
        break;
      uint8_t idx = strchr(filepath, '/') - filepath;
      if (idx > 12)
        idx = 12;
      char subdirname[13];
      strncpy(subdirname, filepath, idx);
      subdirname[idx] = 0;
      subdir->close();
      if (!subdir->open(parent, subdirname, O_READ))
        return SdFile();
      filepath += idx;
      parent->close();
      SdFile *t = parent;
      parent = subdir;
      subdir = t;
    }
    *index = (int)(filepath - origpath);
    return *parent;
  }

  File SDClass::open(const char *filepath, uint8_t mode)
  {
    int pathidx;
    SdFile parentdir = getParentDir(filepath, &pathidx);
    filepath += pathidx;
    if (!filepath[0])
      return File(parentdir, "/");
    SdFile file;
    if (!parentdir.isOpen())
      return File();
    if (!file.open(parentdir, filepath, mode))
      return File();
    parentdir.close();
    if ((mode & (O_APPEND | O_WRITE)) == (O_APPEND | O_WRITE))
      file.seekSet(file.fileSize());
    return File(file, filepath);
  }

  bool SDClass::exists(const char *filepath)
  {
    return walkPath(filepath, root, callback_pathExists);
  }

  bool SDClass::mkdir(const char *filepath)
  {
    return walkPath(filepath, root, callback_makeDirPath);
  }

  bool SDClass::rmdir(const char *filepath)
  {
    return walkPath(filepath, root, callback_rmdir);
  }

  bool SDClass::remove(const char *filepath)
  {
    return walkPath(filepath, root, callback_remove);
  }

  File File::openNextFile(uint8_t mode)
  {
    dir_t p;
    while (_file->readDir(&p) > 0)
    {
      if (p.name[0] == DIR_NAME_FREE)
        return File();
      if (p.name[0] == DIR_NAME_DELETED || p.name[0] == '.')
        continue;
      if (!DIR_IS_FILE_OR_SUBDIR(&p))
        continue;
      SdFile f;
      char name[13];
      _file->dirName(p, name);
      if (f.open(_file, name, mode))
        return File(f, name);
      else
        return File();
    }
    return File();
  }

  void File::rewindDirectory(void)
  {
    if (isDirectory())
      _file->rewind();
  }

  SDClass SD;
};
