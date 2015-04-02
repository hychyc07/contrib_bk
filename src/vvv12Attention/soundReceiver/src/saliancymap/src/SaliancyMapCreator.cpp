/*
 * SaliancaMapCreator.cpp
 *
 *  Created on: Jul 26, 2012
 *      Author: cdondrup
 */

#include "iCub/SaliancyMapCreator.h"

ImageOf<PixelMono> SaliancyMapCreator::createMap(int direction, int volume,
		int distance) {
	ImageOf<PixelMono>* map = new ImageOf<PixelMono>();
	map->resize(320, 240);
	for (int i = 0; i < map->width(); i++) {
		for (int j = 0; j < map->height(); j++) {
			map->pixel(i, j) = PixelMono(0);
//			if (direction == 0) {
//				if (i == (map->width() / 8) * 5 && j == map->height() / 2) {
//					map->pixel(i, j) = PixelMono(distance);
//				}
//			} else if (direction == 1) {
//				if (i == (map->width() / 8 * 3) && j == map->height() / 2) {
//					map->pixel(i, j) = PixelMono(distance);
//				}
//			}
		}
	}
	if(direction == 0)
		map->pixel(map->width()/2+5,map->height()/2)= PixelMono(distance);
	else if (direction == 1)
		map->pixel(map->width()/2-5,map->height()/2)= PixelMono(distance);
	glob_map = map;
	return *map;
}

void SaliancyMapCreator::freeMap() {
	delete glob_map;
	glob_map = NULL;
}

