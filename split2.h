/*
 * split2.h
 *
 *  Created on: 17 Jan 2020
 *      Author: pgm
 *  ref and thanks:
 *  http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
 */

#ifndef SPLIT2_H_
#define SPLIT2_H_

#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>



template <class Container>
void split2(const std::string& str, Container& cont, char delim = ' ')
{
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delim)) {
        cont.push_back(token);
    }
}


#endif /* SPLIT2_H_ */
