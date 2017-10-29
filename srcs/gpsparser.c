#include<iostream>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <iomanip>
#include "gpsparser.h"

using namespace std;


bool gpsparser(char* data, double* lon, double* lat, double* HDOP, int* numSV) {
	string const str(data);
	string slon, slat, sHDOP, snumSV, sposMode, snavStatus, checksum; //sposMode, snavStatus for future useage;
	string const header("GNGNS");
	char cs;
	unsigned int cnt = str.find(header);
	if (cnt > str.size()){
		cerr<<"No header!!!"<<endl;
		return false;
	}
	else
		cnt += 6; // $GNGPS,
	cs = 0 ^'G' ^ 'N' ^ 'G' ^ 'N' ^ 'S' ^ ',';

	int comma = 0;
	while (cnt < str.size() && str[cnt] != '\r' && str[cnt]!='\n') {
		switch (str[cnt]) {
		case ',': ++comma;
			break;
		case '*': ++comma;
			break;
		default:
			switch (comma) {
				//case 0: //UTC
			case 1: slat += str[cnt];
				break;
			case 3: slon += str[cnt];
				break;
			case 5: sposMode += str[cnt];
				break;
			case 6: snumSV += str[cnt];
				break;
			case 7: sHDOP += str[cnt];
				break;
			case 11: snavStatus += str[cnt];
				break;
			case 12: checksum += str[cnt];
			default: break;
			}
			break;
		}
		if (comma < 12)
			cs ^= str[cnt];
		++cnt;
	}
	//safe check sum
	if (checksum.size() != 2) {
		cerr << "checksum error: len" << endl;
		return false;
	}
	char check = 0, temp;
	for (int i = 0; i < 2; ++i) {
		if (checksum[i] <= '9' && checksum[i] >= '0')
			temp = checksum[i] - '0';
		else if (checksum[i] <= 'F' && checksum[i] >= 'A')
			temp = checksum[i] - 'A' + 10;
		else if (checksum[i] <= 'f' && checksum[i] >= 'a')
			temp = checksum[i] - 'a' + 10;
		else {
			cerr << "checksum error: invalid arguments" << endl;
			return false;
		}
		check = check * 16 + temp;
	}
	if (cs != check) {
		cerr << "checksum failed: " << check << " " << cs << endl;
		return false;
	}

	//processing data;
	string str1 = slat.substr(0, 2), str2 = slon.substr(0, 3);
	slat.erase(0, 2);
	slon.erase(0, 3);
	double prelat, prelon;
	*HDOP = 0;
	*numSV = 0; // 0 denote HDOP&numSV is missed in data
	try {
		prelat = stod(str1);
		*lat = stod(slat); //mm.mmmmm
		prelon = stod(str2);
		*lon = stod(slon); //mm.mmmmm
		*lat = prelat + *lat / 60; // 5 valid decimal fraction 
		*lon = prelon + *lon / 60; // 5 valid decimal fraction 
		if (!sHDOP.empty())
			*HDOP = stod(sHDOP);
		if (!snumSV.empty())
			*numSV = stoi(snumSV);
	}
	catch (invalid_argument err) {
		cout << err.what() << ": invalid_argument" << endl;
		return false;
	}
	return true;
}

//// gpsparser test
//int main() {
//	char test[] = "$$GNGNS,154557.10,3203.24419,N,11846.58214,E,AA,07,1.39,50.1,2.4,,*68";
//	double lat, lon, HDOP;
//	int numSV;
//	if (gpsparser(test, &lon, &lat, &HDOP, &numSV))
//		cout << setiosflags(ios::fixed) << setprecision(5) << "test passed!\n" << "lon: " << lon
//			<< "\tlat: " << lat << "\tHDOP: " << HDOP << "\tnumSV: " << numSV << endl;
//	else
//		cout << "parse error…… try again" << endl;
//	return 0;
//}
