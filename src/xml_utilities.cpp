/*
 * xml_utilities.cpp
 *
 *  Created on: Dec 1, 2011
 *      Author: aalbiol
 */

#include "xml_utilities.h"
// ***************** AUXILIARY FUNCTIONS *********************************
xmlNode * GetSon(xmlNode *node, char *name){

	xmlNode *cur_node = NULL;
	for (cur_node = node->children; cur_node; cur_node = cur_node->next)
	{
		if (cur_node->type == XML_ELEMENT_NODE)
		{
			if (strcmp(name,(char *) cur_node->name)==0)
			{
				return cur_node;
			}
		}
	}
	return NULL;
}

int NumberOfSons(xmlNode *node)
{
  xmlNode *cur_node = NULL;
  int k=0;

  for (cur_node = node->children; cur_node; cur_node = cur_node->next)
  {
    if (cur_node->type == XML_ELEMENT_NODE)
	  {
	    k++;
	  }
  }
  return k;
}


// Version con retornos de carros
//std::string getXmlItem(std::istream& MyInput,std::string tag)
//{
//  std::string BeginTag="<"+tag+">";
//  std::string EndTag="</"+tag+">";
//  std::string output("");
//  
//  if (searchWord(MyInput,BeginTag))
//  {
//    int BeginPos=MyInput.tellg();
//    
//    if (searchWord(MyInput,EndTag))
//    {
//      int EndPos=MyInput.tellg();
//      EndPos+= EndTag.size();
//      int length=EndPos-BeginPos+1;
//      
//      MyInput.seekg(BeginPos,std::ios::beg);
//      
//      char *buffer = new char [length+1];
//      
//      // read data as a block:
//      MyInput.read (buffer,length);
//      buffer[length]=0;
//      output=buffer;
//      
//      delete[] buffer;
//      return output;
//    }
//  }
//  
//  
//  return output;
//}

std::string getXmlItem(std::istream& MyInput,std::string tag)
{
  std::string BeginTag="<"+tag+">";
  std::string EndTag="</"+tag+">";
  std::string word;
  
  while(!MyInput.eof())
  {
    MyInput>>word;
    
    if (word==BeginTag)
    {
      break;
    }
  }
  if (MyInput.eof())
    return ""; 
  
  
  std::string output(BeginTag);
  
  while(!MyInput.eof())
  {
    MyInput>>word;
    
    if (word!=EndTag)
    {
      output += " " + word;
    }
    else {
      output += " " + EndTag;
      return output;
    }
    
  }
  
  return "";
  
}

