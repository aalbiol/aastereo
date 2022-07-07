#include <string.h>
#include <sys/stat.h>

#include <fstream>
#include <stdlib.h>
#include <iostream>
#include <string>
#include <locale>
#include <vector>
#include <algorithm>
#include <set>
#include <string>

#include "file_utilities.h"

#include <stdlib.h>


namespace upvsoft{
  namespace misc{
    namespace fileutilities {

      bool FileExists(const std::string &strFilename) {
        struct stat stFileInfo;
        bool blnReturn;
        int intStat;

        // Attempt to get the file attributes
        intStat = stat(strFilename.c_str(),&stFileInfo);
        if(intStat == 0) {
            // We were able to get the file attributes
            // so the file obviously exists.
            blnReturn = true;
        } else {
            // We were not able to get the file attributes.
            // This may mean that we don't have permission to
            // access the folder which contains this file. If you
            // need to do that level of checking, lookup the
            // return values of stat which will give you
            // more details on why stat failed.
            blnReturn = false;
        }

        return(blnReturn);
      }

      void mkDir_P(const char *dir) {

        char *tmp = strdup(dir);

        char *p = NULL;



        size_t len = strlen(tmp);
        if(tmp[len - 1] == '/')
          tmp[len - 1] = 0;
        for(p = tmp + 1; *p; p++)
          if(*p == '/') {
              *p = 0;
              mkdir(tmp, S_IRWXU);
              *p = '/';
          }
        mkdir(tmp, S_IRWXU);

        if (tmp!=NULL)
          free(tmp);
      }
      void mkDir_P(const std::string &dir)
      {
        mkDir_P(dir.c_str());
      }


      using namespace std;

      /**
       * basename_loc returns the basename start position and extension position.
       * It is advisable not to use it directly to get basename and extension.
       * Use the provided functions for this purpose.
       *
       * @return pair<startpos,extpos>
       */
      std::pair<std::string::size_type, std::string::size_type>
      basename_loc(
          const std::string& fullpath,
          const std::string ext = ".");

      pair<string::size_type, string::size_type>
      basename_loc(
          const string& fullpath,
          const string ext)
          {// {{{
        // set not to remove extension
        string::size_type dotpos = string::npos;
        if (ext==".")
          dotpos = fullpath.find_last_of(ext);
        else if (!ext.empty()) {
            dotpos = fullpath.rfind(ext);
            if (dotpos+ext.length()!=fullpath.length())
              dotpos = string::npos;
        }

        // if given extension string (for e.g. ".png") precisely
        // operate like `basename` command on unix
        // i.e. string must end with ext
        if (ext.length() >1 &&
            dotpos != string::npos &&
            (fullpath.length() - ext.length()) != dotpos)
          {
            dotpos = string::npos;
          }

        string::size_type startpos = fullpath.find_last_of('/');

        // if no forward slash, search for backward slash
        if (startpos == string::npos)
          startpos = fullpath.find_last_of('\\');

        if (startpos == string::npos)
          startpos = 0;
        else {
            // check if start pos is latter than dotpos
            // if yes, this implies that dot pos is not valid

            // size_type is probably unsigned, difference operator
            // for unsigned keeps string unsigned. this is bad
            // so cast to signed long int for conversion
            if (dotpos != string::npos &&
                static_cast<long >(startpos) - static_cast<long>(dotpos) > 0)
              dotpos = string::npos;
            ++startpos;
        }
        return make_pair(startpos,dotpos);
          }// }}}

      string basename(
          const string& fullpath,
          const string ext)
      {
        string dot_ext = "." + ext;
        pair<
        string::size_type,
        string::size_type> loc = basename_loc(fullpath,dot_ext);

        return fullpath.substr(loc.first, loc.second - loc.first);
      }
      string dirname(
          const string& fullpath)
      {
        pair<
        string::size_type,
        string::size_type> loc = basename_loc(fullpath);

        if (loc.first>0)
          return fullpath.substr(0,loc.first-1);
        else
          return string();
      }
      string extension( const string& fullpath )
      {
        const string ext(".");
        pair<
        string::size_type,
        string::size_type> loc = basename_loc(fullpath,ext);

        if (loc.second != string::npos){
            if (loc.second +1 == fullpath.length() && ext!=".")
              return fullpath.substr(loc.second);
            else
              return fullpath.substr(loc.second+1);
        }
        return string();
      }


      std::string change_extension( const std::string& fullpath, const std::string& old_extension, const std::string& new_extension )
      {
        size_t found= fullpath.find(old_extension);

        std::string new_name;

        if (found!=std::string::npos)
          { // replace extension
            new_name = fullpath;
            new_name.replace(found, old_extension.length(),new_extension);
          }
        else
          { // add extension if not found
            new_name = fullpath + std::string(".") + new_extension;
          }
        return new_name;


      }


    }

  }
}

namespace aautilities {
using namespace std;
static inline std::string &rtrim(std::string &s) {
	//s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	std::size_t found = s.find_last_not_of(" \n\r\t");
	if(found != std::string::npos)
		s.erase(found + 1);
	return s;
}

// trim from end
static inline std::string &ltrim(std::string &s) {
//	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
	std::size_t found = s.find_first_not_of(" \n\r\t");
	if(found != std::string::npos)
		s.erase(0,found);
	return s;
}

// trim from both ends
static inline std::string &trim(std::string &s) {
	return ltrim(rtrim(s));
}



set< string>  isVideos(std::string & filename) {
	string extension = get_extension(filename);
	locale loc;

	// Convert to lowercase
	for (size_t i=0; i<extension.size(); ++i)
		extension[i] = tolower(extension[i],loc);

	set<string> extensions;
	//Add as many extensions as wanted
	extensions.insert("avi");
	extensions.insert("mpg");
	extensions.insert("mpeg");
	extensions.insert("mov");
	extensions.insert("mp4");
	extensions.insert("m4v");
	extensions.insert("mkv");


	bool is_Video = extensions.count(extension);
	std::set <string> videonames;

	if( is_Video ) {
		videonames.insert(filename);
		return videonames;
	}

	//Here it is a list file.
	fstream filestr;

	try {
		filestr.open (filename.c_str(),fstream::in);
	}
	catch(...) {
		std::cerr << "Could not open " << filename << "\n";
	}

	// >> i/o operations here <<



	if(! filestr.is_open() ) {
		std::cerr << "Could not open " << filename << "\n";
		return videonames; //Empty
	}
	std::string name;
	while( ! filestr.eof() ) {
		filestr >> name ;
		name = trim( name ); // Remove white space from the beginning and end
		extension = get_extension(name);
		// Convert to lowercase
		for (size_t i=0; i<extension.size(); ++i)
			extension[i] = tolower(extension[i],loc);

		if( extensions.count(extension) ) {
			videonames.insert(name);
		}

	}
	filestr.close();
	return videonames;
}


int isCamera(std::string &input) {
	int length = input.size();
	if( length != 1 )
		return -1;

	if (! isdigit( input[0] ) )
		return -1;

	return  atoi(input.c_str() );
}


std::string get_extension(std::string & filename) {
	std::string extension;
	int position = filename.find_last_of(".");

	if(position ==  string::npos )
		return extension;
	extension = filename.substr(position + 1);
	return extension;
}


std::string get_path(std::string & filename) {

	std::string path;
	int found = filename.find_last_of("/\\");
	if (found == string::npos)
		return path;
	path = filename.substr(0,found);
	return path;
}


std::string get_basename(std::string & filename) {

	std::string basename;
	int found = filename.find_last_of("/\\");
	if (found == string::npos)
		return basename;
	basename = filename.substr(found+1);
	return basename;
}


bool isVideo(std::string & filename) {
	string extension = get_extension(filename);
	locale loc;

	// Convert to lowercase
	for (size_t i=0; i<extension.size(); ++i)
		extension[i] = tolower(extension[i],loc);

	std::set<string> extensions;
	//Add as many extensions as wanted
	extensions.insert("avi");
	extensions.insert("mpg");
	extensions.insert("mpeg");
	extensions.insert("mov");
	extensions.insert("mp4");
	extensions.insert("m4v");
	extensions.insert("mkv");

	return extensions.count(extension);
}


set< string>  isImages(std::string & filename) {
	string extension = get_extension(filename);
	locale loc;

	// Convert to lowercase
	for (size_t i=0; i<extension.size(); ++i)
		extension[i] = tolower(extension[i],loc);

	set<string> extensions;
	//Add as many extensions as wanted
	extensions.insert("jpg");
	extensions.insert("jpeg");
	extensions.insert("tiff");
	extensions.insert("tif");
	extensions.insert("pgm");
	extensions.insert("pbm");
	extensions.insert("bmp");


	bool is_Image = extensions.count(extension);
	std::set <string> imagenames;

	if( is_Image ) {
		imagenames.insert(filename);
		return imagenames;
	}

	//Here it is a list file.
	fstream filestr;

	try {
		filestr.open (filename.c_str(),fstream::in);
	}
	catch(...) {
		std::cerr << "Could not open " << filename << "\n";
	}

	// >> i/o operations here <<



	if(! filestr.is_open() ) {
		std::cerr << "Could not open " << filename << "\n";
		return imagenames; //Empty
	}
	std::string name;
	while( ! filestr.eof() ) {
		filestr >> name ;
		name = trim( name ); // Remove white space from the beginning and end
		extension = get_extension(name);
		// Convert to lowercase
		for (size_t i=0; i<extension.size(); ++i)
			extension[i] = tolower(extension[i],loc);

		if( extensions.count(extension) ) {
			imagenames.insert(name);
		}

	}
	filestr.close();
	return imagenames;
}

}
// esta funcion es muy machine independent pero requiere boost
//bool verifydir(const string outfile, const bool isdir)
//{
//    // check if outimage is valid
//    namespace fs=boost::filesystem;
//    fs::path outdir(outfile,fs::native);
//    if (!isdir)
//        return ((fs::exists(outdir) && fs::is_directory(outdir))? false:true);
//    return ((fs::exists(outdir) && fs::is_directory(outdir))
//            || fs::create_directory(outdir));
//    /*try {
//    }catch (exception e) {
//        return false;
//    }*/
//}
