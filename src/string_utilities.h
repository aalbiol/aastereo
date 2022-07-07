/*
 *  string_utilities.h
 *  ProjectXcode
 *
 *  Created by Alberto Albiol on 5/6/10.
 *  Copyright 2010 Universidad Politecnica Valencia. All rights reserved.
 *
 */

#ifndef D_STRING_UTILITIES_H
#define D_STRING_UTILITIES_H
#include <string>
#include <sstream>
#include <vector>
#include <fstream>
#include <iostream>


namespace upvsoft
{
  namespace misc
  {
    namespace stringutilities
    {
      //! Divide un string en pedazos. Los puntos de ruptura pueden ser cualquiera de los caracteres de delimiters
      /**
 \param string de entrada que vamos a dividir
 \param vector con cada uno de los trozos
 \param string en el que cada caracter indica un posible separador
 \return void
       **/
      void Tokenize(const std::string& str,std::vector<std::string>& tokens,const std::string& delimiters);


      //! Compara dos strings, de forma case-insensitive (mayusculas,minusculas)
      /**
 \return -1,0 or 1 according to strings' lengths
       **/
      int Nocase_cmp(const std::string & s1, const std::string& s2);

      //! Searchs line in text file, the file pointer point at the begin of the found line
      /**
 \return true if the line is found
       **/
      bool searchLine(std::istream& MyInput,std::string SearchedLine);
      //! Searchs word in text file, the file pointer point at the begin of the found word
      /**
 \return true if the word is found
       **/
      bool searchWord(std::istream& MyInput,std::string SearchedWord);


      /**
 \ Removes initial and trailing whitespaces from string. Keeps intermediate ones.
       **/
      std::string trim(const std::string & input);

      //! Parses a text file.
      /**
 \param name of the text file
 \return vector of size number of lines, and for each line there is a vector with the words
       **/
      std::vector<   std::vector<std::string>   > readList(std::string filename);

      //! Convert value to string.
      /**
 \param T: variable to store the result
 \param string: string to convert
 \param ios_base: one of std::hex, std::dec or std::oct
 \return true if the conversion was correct
 \example: from_string<float>(f, std::string("123.456"), std::dec)
       **/
      template <class T>
      bool from_string(T& t, const std::string& s, std::ios_base& (*f)(std::ios_base&))
      {
        std::istringstream iss(s);
        return !(iss >> f >> t).fail();
      }


    }
  }
}
#endif 
