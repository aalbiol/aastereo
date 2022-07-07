/*
 *  File_utilities.h
 *  ProjectXcode
 *
 *  Created by Alberto Albiol on 7/3/11.
 *  Copyright 2010 Universidad Politecnica Valencia. All rights reserved.
 *
 */

#ifndef D_FILE_UTILITIES_H
#define D_FILE_UTILITIES_H

#include <string>
#include <set>


namespace upvsoft{
  namespace misc{
    namespace fileutilities {

        
        //! Checks if file exists
        /**
         \param fullpath
         \param old extension
         \param new extension
         **/
        std::string change_extension( const std::string& fullpath, const std::string& old_extension, const std::string& new_extension );
        
        
        //! Checks if file exists
        /**
         \param File name 
         **/
        bool FileExists(const std::string &strFilename);
        
        //! Creates directory, equivalent to mkdir -p
        /**
         \param directory name
         \return void
         **/
        void mkDir_P(const char *dir);
        
        //! Creates directory, equivalent to mkdir -p
        /**
         \param directory name
         \return void
         **/
        void mkDir_P(const std::string &dir);
        
        //! extracts basename from full path
        /**
         \param full path filename /tmp/test.txt -> test.txt
         \param if a extension is given then it is also removed from fullpath, /tmp/test.txt -> test
         \return basename
         **/
        std::string basename( const std::string& fullpath, const std::string ext="");
        
        //! extracts directory from full path
        /**
         \param full path filename /tmp/test.txt -> /tmp
         \return directory
         **/
        std::string dirname( const std::string& fullpath);
        
        //! extracts extension from fullpath, can be the second input argument of basename function
        /**
         \param full path filename /tmp/test.txt -> txt
         \return directory
         **/
        std::string extension( const std::string& fullpath);
        




    }
  }
}
namespace aautilities {

std::string get_extension(std::string & filename);
std::string get_path(std::string & filename);
std::string get_basename(std::string & filename);

int isCamera(std::string &input);
bool isVideo(std::string & filename);
std::set< std::string>  isImages(std::string & filename);
std::set< std::string>  isVideos(std::string & filename);
}
#endif 
