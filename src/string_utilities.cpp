/*
 *  string_utilities.cpp
 *  ProjectXcode
 *
 *  Created by Alberto Albiol on 5/6/10.
 *  Copyright 2010 Universidad Politecnica Valencia. All rights reserved.
 *
 */
#include <string.h>
#include "string_utilities.h"


#include <iostream>



namespace upvsoft {
  namespace misc
  {
    namespace stringutilities
    {
      using namespace std;

      void Tokenize(const string& str,
          vector<string>& tokens,
          const string& delimiters)
      {
        // Skip delimiters at beginning.
        string::size_type lastPos = str.find_first_not_of(delimiters, 0);
        // Find first "non-delimiter".
        string::size_type pos     = str.find_first_of(delimiters, lastPos);

        tokens.clear();
        while (string::npos != pos || string::npos != lastPos)
          {
            // Found a token, add it to the vector.
            tokens.push_back(str.substr(lastPos, pos - lastPos));
            // Skip delimiters.  Note the "not_of"
            lastPos = str.find_first_not_of(delimiters, pos);
            // Find next "non-delimiter"
            pos = str.find_first_of(delimiters, lastPos);
          }
      }


      int Nocase_cmp(const string & s1, const string& s2)
      {
        string::const_iterator it1=s1.begin();
        string::const_iterator it2=s2.begin();

        //stop when either string's end has been reached
        while ( (it1!=s1.end()) && (it2!=s2.end()) )
          {
            if(::toupper(*it1) != ::toupper(*it2)) //letters differ?
              // return -1 to indicate smaller than, 1 otherwise
              return (::toupper(*it1)  < ::toupper(*it2)) ? -1 : 1;
            //proceed to the next character in each string
            ++it1;
            ++it2;
          }
        size_t size1=s1.size(), size2=s2.size();// cache lengths
        //return -1,0 or 1 according to strings' lengths
        if (size1==size2)
          return 0;
        return (size1<size2) ? -1 : 1;
      }



      bool searchLine(std::istream& MyInput,std::string SearchedLine)
      {
        std::string line;
        while(!MyInput.eof())
          {
            getline(MyInput,line);
            if (line==SearchedLine)
              {
                MyInput.seekg(- SearchedLine.size(),std::ios::cur);
                return true;
              }
          }
        return false;
      }

      bool searchWord(std::istream& MyInput,std::string SearchedWord)
      {
        std::string word;
        while(!MyInput.eof())
          {
            MyInput>>word;

            if (word==SearchedWord)
              {

                MyInput.seekg(- SearchedWord.size(),std::ios::cur);
                return true;
              }
          }
        return false;
      }


      //Remove white spaces from beginning and end of a string
      std::string trim(const std::string & input)
      {

        std::string Input = input;
        const char * s= Input.c_str();
        int length = Input.size();
        int k;
        for(k=length-1; k>=0; k--)
          if(s[k] != ' ' && s[k] != '\t' && s[k]!='\n' && s[k]!='\r')
            break;

        int end_position = k;



        length=strlen(s);
        for(k=0; k<length; k++)
          if(s[k] != ' ' && s[k] != '\t' && s[k]!='\n' && s[k]!='\r')
            break;
        int start_position = k;

        std::string res = Input.substr(start_position,end_position-start_position+1);

        //  std::cerr<<"Trim  Input =*"<<input<<"*  Output = *"<<res<<"*\n";
        return res;




      }

      std::vector<   std::vector<std::string>   > readList(std::string filename){
        std::vector<   std::vector<std::string>   > parsedFile;
        std::ifstream file(filename.c_str());
        if(!file.is_open()){
            std::cout << "File " << filename << " not opened..." << std::endl;
            return parsedFile;
        }

        int k=0;
        while(!file.eof()){
            std::vector<std::string> words;
            std::string line;
            std::string word;
            getline(file,line);
            k++;
            Tokenize(line, words," ");
            if (words.size()>0)
              parsedFile.push_back(words);
        }
        file.close();
        return parsedFile;
      }

    }
  }
}
