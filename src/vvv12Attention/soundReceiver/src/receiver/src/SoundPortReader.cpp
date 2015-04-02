/*
 * SoundPortReader.cpp
 *
 *
 *      Author: Christian Dondrup
 */

#include "iCub/SoundPortReader.h"

bool SoundPortReader::threadInit() {
	stoploop = false;
	output.resize(0, 0);
	signaleProcessor = new SignalProcessor(highPassFreq);
	volume = new Volume(threshold);
	return true;
}

void SoundPortReader::run() {
	cout << " SoundPortReader" << endl;
	while (!stoploop) {
		sound = in->read();
		//Checking if the bottle is good
		if (sound != NULL) {
//			cout << "Freq: " << sound->getFrequency() << ", Bytes: "
//					<< sound->getBytesPerSample() << ", Chan: "
//					<< sound->getChannels() << ", Samples: "
//					<< sound->getSamples() << endl;

//Fill buffer
			if (buffer.size() < BUFFER_SIZE) {
				buffer.push_back(*sound);
			} else {
				buffer.erase(buffer.begin());
				buffer.push_back(*sound);
			}

			//Create sound window
			Sound volume_buffer;
			volume_buffer.resize(buffer.size() * sound->getSamples(), 2);
			for (int i = 0; i < buffer.size(); i++) {
				Sound* tmp = &buffer[i];
				for (int j = 0; j < tmp->getSamples(); j++) {
					volume_buffer.setSafe(tmp->getSafe(j, 0),
							j + i * tmp->getSamples(), 0);
					volume_buffer.setSafe(tmp->getSafe(j, 1),
							j + i * tmp->getSamples(), 1);
				}
			}

			//Filter window
			float** signal = new float*[2];

			signal[0] = new float[volume_buffer.getSamples()];
			signal[1] = new float[volume_buffer.getSamples()];
			for (int i = 0; i < volume_buffer.getSamples(); i++) {
				signal[0][i] = pcmToFloat(volume_buffer.getSafe(i, 0));
				signal[1][i] = pcmToFloat(volume_buffer.getSafe(i, 1));
				if (signal[0][i] > 1.0 || signal[0][i] < -1.0
						|| signal[1][i] > 1.0 || signal[1][i] < -1.0) {
					cerr << "++++++Wrong transformation+++++++";
				}
			}
			float** filtered = new float*[2];
			filtered[0] = signaleProcessor->process(signal[0],
					volume_buffer.getSamples());
			filtered[1] = signaleProcessor->process(signal[1],
					volume_buffer.getSamples());
			for (int i = 0; i < volume_buffer.getSamples(); i++) {
				volume_buffer.setSafe(floatToPcm(filtered[0][i]), i, 0);
				volume_buffer.setSafe(floatToPcm(filtered[1][i]), i, 1);
			}

//			Convert
			float** samples = new float*[2];
			samples[0] = new float[volume_buffer.getSamples()];
			samples[1] = new float[volume_buffer.getSamples()];
			for (int i = 0; i < volume_buffer.getSamples(); i++) {
				samples[0][i] = pcmToFloat(volume_buffer.getSafe(i, 0));
				if (volume_buffer.getChannels() == 2) {
					samples[1][i] = pcmToFloat(volume_buffer.getSafe(i, 1));
				}
			}

			//Correlate
			Correlator correlator;
			float** envelope = new float*[2];
			float** onsets = new float*[2];
			envelope[0] = correlator.createEnvelope(samples[0],
					volume_buffer.getSamples());
			onsets[0] = correlator.createOnsets(envelope[0],
					volume_buffer.getSamples());
			if (volume_buffer.getChannels() == 2) {
				envelope[1] = correlator.createEnvelope(samples[1],
						volume_buffer.getSamples());
				onsets[1] = correlator.createOnsets(envelope[1],
						volume_buffer.getSamples());
			}
			int distance = correlator.crossCorrelate(onsets,
					volume_buffer.getSamples());
//			for (int i = 0; i < sound->getSamples(); i++)
//				cout << samples[0][i] << " -> " << envelope[0][i] << "\t->\t"
//						<< onsets[0][i] << endl;
			float angle = correlator.getAngle(distance, sound->getFrequency());
//			cout << "Distance: " << distance << " -> " << angle << endl;

			for (int i = 0; i < 2; i++) {
				delete[] envelope[i];
				delete[] onsets[i];
				delete[] samples[i];
				delete[] signal[i];
				delete[] filtered[i];
			}
			delete[] envelope;
			delete[] onsets;
			delete[] samples;
			delete[] signal;
			delete[] filtered;

//			for(int i = 0; i < check.getSamples(); i++) {
//				cout<<"Right >>>> "<<check.getSafe(i,0)<<" -> "<<signal[0][i]<<" -> "<<volume_buffer.getSafe(i,0)<<endl;
//				cout<<"Left  >>>> "<<check.getSafe(i,1)<<" -> "<<signal[1][i]<<" -> "<<volume_buffer.getSafe(i,1)<<endl;
//			}

//Check volume
			int direction = volume->getLoudestChannel(volume_buffer);
			switch (direction) {
			case -1:
				cout << "Center" << endl;
				break;
			case 0:
				cout << "---------Right :" << volume->getDistance() << endl;
				break;
			case 1:
				cout << "+++++++++Left  :" << volume->getDistance() << endl;
				break;
			default:
				cout << "Unknown" << endl;
				break;
			}

			if (direction >= 0) {
				out->prepare() = saliancyMapCreator.createMap(direction,
						volume->getVolume(volume_buffer, direction),
						volume->getDistance());
				out->write();
				saliancyMapCreator.freeMap();
			}

//			if (buffer.size() == BUFFER_SIZE) {
//				for (int i = BUFFER_SIZE / 2; i < sound->getSamples(); i++) {
//					sound->setSafe(volume_buffer.getSafe(i, 0), i, 0);
//					sound->setSafe(volume_buffer.getSafe(i, 1), i, 1);
//				}
//			}
//			//Save if specified
//			if (filename != "") {
//				if (output.getSamples() == 0) {
//					output = *sound;
//				} else {
//					int i = output.getSamples();
////					cout << i << endl;
//					Sound tmp = output;
//					output.resize(sound->getSamples() + i,
//							sound->getChannels());
//					for (int k = 0; k < i; k++) {
//						output.setSafe(tmp.getSafe(k, 0), k, 0);
//						output.setSafe(tmp.getSafe(k, 1), k, 1);
//					}
////					cout << output.getSamples() << endl;
//					int j = 0;
//					for (; i < output.getSamples(); i++) {
//						output.setSafe(sound->getSafe(j, 0), i, 0);
//						output.setSafe(sound->getSafe(j, 1), i, 1);
//						j++;
//					}
//				}
//			}
		}
	}
}

void SoundPortReader::threadRelease() {
	if (filename != "") {
		cout << "Writing to: " << filename << endl;
		cout << "Writing " << output.getSamples() << " samples." << endl;
		if (!file::write(output, filename.c_str())) {
			cerr << "ERROR: Unable to write!" << endl;
		}
	}
	delete signaleProcessor;
	delete volume;
	cout << "  SoundPortReader stopped" << endl;
}

