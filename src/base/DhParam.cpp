#include "DhParam.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include "base/Math.hpp"

using namespace std;

std::vector<DhParam> loadDhParamsFromFile(const std::string filename)
{
	ifstream file_input(filename, ios::in);
	string line;

	std::vector<DhParam> params;

	while (getline(file_input, line)) {
		if (line[0] == '#') continue;

		stringstream ss(line);
		DhParam p;
		string sink;
		string alpha_str;

		ss >> p.sigma;

		if (p.sigma == "R") {
			ss >> p.a >> p.d >> alpha_str >> sink;
			float result;

			float operand1, operand2;
			char operation;
			size_t operator_loc;

			auto set_operands = [&](const char operation) {
				if (operator_loc != string::npos) {
					string before = alpha_str.substr(0, operator_loc);
					string after = alpha_str.substr(operator_loc + 1, alpha_str.length() - operator_loc);

					if (before.find("pi") != string::npos) {
						operand1 = FW_PI;
						operand2 = std::stof(after);
					}
					else if (after.find("pi") != string::npos) {
						operand1 = std::stof(before);
						operand2 = FW_PI;
					}
					else {
						cout << "wtf!!! " << endl;
					}
				}
			};
			if ((operator_loc = alpha_str.find('*')) != string::npos) {
				set_operands('*');
				result = operand1 * operand2;
			}
			else if ((operator_loc = alpha_str.find('/')) != string::npos) {
				set_operands('/');
				result = operand1 / operand2;
			}
			else {
				result = 0.0f;
			}

			p.alpha = result;
			p.theta = -1;
		}
		else if (p.sigma == "P") {
			ss >> p.a >> sink >> p.alpha >> p.theta;
			p.d = -1;
		}
		else {
			cout << "Invalid joint type: " << p.sigma << endl;
			continue;
		}

		params.push_back(p);
	}
	return params;
}