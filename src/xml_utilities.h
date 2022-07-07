/*
 * xml_utilities.h
 *
 *  Created on: Dec 1, 2011
 *      Author: aalbiol
 */

#ifndef XML_UTILITIES_H_
#define XML_UTILITIES_H_
#include <string.h>
#include <libxml/parser.h>
#include <libxml/tree.h>
#include "string_utilities.h"

xmlNode * GetSon(xmlNode *node, char *name);
int NumberOfSons(xmlNode *node);

//! Get value of node
/**
 \param node: xml node
 \return: the value of the node.
**/
template <class T>
T getValueXmlNode(xmlNode *node)
{
	T out;
	char *buff = (char *) xmlNodeGetContent(node);
	if(!upvsoft::misc::stringutilities::from_string<T>(out, std::string(buff), std::dec)) {
		std::cout << "Can't read value from node" << std::endl;
		exit(-1);
	}
	free(buff);
	return out;
}

//! Extracts a xmlitem from a file
/**
 \param input file
 \param tag of the xml item without  "<", and ">" 
**/
/**
 \return the item or empty string
 **/
std::string getXmlItem(std::istream& MyInput,std::string tag);

#endif /* XML_UTILITIES_H_ */
