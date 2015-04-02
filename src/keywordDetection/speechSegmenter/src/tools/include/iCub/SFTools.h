/*
 * SFTools.h
 *
 *  Created on: Jul 5, 2012
 *      Author: cdondrup
 */

#ifndef SFTOOLS_H_
#define SFTOOLS_H_

#define SF_CONTAINER(x)		((x) & SF_FORMAT_TYPEMASK)
#define SF_CODEC(x)			((x) & SF_FORMAT_SUBMASK)
#define SF_ENDIAN(x)		((x) & SF_FORMAT_ENDMASK)


#endif /* SFTOOLS_H_ */
