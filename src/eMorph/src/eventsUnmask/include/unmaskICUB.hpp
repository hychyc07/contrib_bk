/* 
 * Copyright (C) <year> RobotCub Consortium, European Commission FP6 Project IST-004370
 * Author Charles Clercq
 * email:   charles.clercq@robotcub.org
 * website: www.robotcub.org
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/
#ifndef UNMASKICUB_HPP
#define UNMASKICUB_HPP

#include "unmask.hpp"

class unmaskICUB:public unmask
{
public:
    unmaskICUB(bool _save=false);
    unmaskICUB(const unmaskICUB&);

    ~unmaskICUB();

    unmaskICUB& operator=(const unmaskICUB&);

	void setBuffer(char*, uint);
    void reshapeBuffer();
    int getUmaskedData(uint&, uint&, int&, uint&, uint&);
    int reset();
private:
    void saveBuffer(char*, uint);
    uint snapBuffer();

    void objcpy(const unmaskICUB&);
    uint *buffer;
    uint *bufSnapShot;
    uint tsPacket;
    bool save;
};

#endif //UNMASKICUB_HPP
