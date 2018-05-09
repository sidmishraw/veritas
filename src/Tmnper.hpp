//
//  Tmnper.hpp
//  Tmnper
//
//  Created by Sidharth Mishra on 2/22/18.
//  Copyright © 2018 Sidharth Mishra. All rights reserved.
//

#pragma once /* ---- include this header file only once ---- */

#ifndef Tmnper_hpp
#define Tmnper_hpp

#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <regex>
#include <string>

namespace Tmnper {

/**
 * Saves the contents into a file with the given extension.
 */
using namespace std;
bool saveIntoTmpr(string fileName, string contents, string extn);

/**
 * Loads the contents from a file with the given extension.
 */
using namespace std;
string loadFromTmpr(string fileName, string extn);
}  // namespace Tmnper

#endif /* Tmnper_hpp */
