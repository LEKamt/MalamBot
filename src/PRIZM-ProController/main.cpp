#include <iostream>
#include <string>
#include <algorithm> // Required for clamp (replaces constrain)

// This removes the need to use std:: everywhere
using namespace std;

int main() {
    string bufferRX = "A1500B-1500C5000D1000E";

    // Declared missing variables
    int servo_cmd = 0;
    int riel_cmd = 0;
    float wl_cmd = 0.0f;
    float wr_cmd = 0.0f;

    // string::npos means "not found" in standard C++
    size_t idxA = bufferRX.find('A');
    size_t idxB = bufferRX.find('B');
    size_t idxC = bufferRX.find('C');
    size_t idxD = bufferRX.find('D');
    size_t idxE = bufferRX.find('E');

    if (idxA != string::npos && idxB != string::npos && 
        idxC != string::npos && idxD != string::npos && idxE != string::npos) {
        
        // Convert substrings to floats (length must be calculated: end - start)
        float raw_wr = stof(bufferRX.substr(idxA + 1, idxB)) / 1000.0f;
        float raw_wl = stof(bufferRX.substr(idxB + 1, idxC)) / 1000.0f;
        float powerRaw = stof(bufferRX.substr(idxC + 1, idxD)) / 1000.0f;
        float servoRaw = stof(bufferRX.substr(idxD + 1, idxE)) / 1000.0f;

        // clamp restricts the value between min and max
        wr_cmd = clamp(raw_wr, -12.566f, 12.566f);
        wl_cmd = clamp(raw_wl, -12.566f, 12.566f);
        riel_cmd = clamp(static_cast<int>(powerRaw), -20, 20);
        servo_cmd = clamp(static_cast<int>(servoRaw), 0, 180);
    }

    // Output results (Fixed from std::out to cout)
    cout << "Wheel Right Command: " << wr_cmd << endl;
    cout << "Wheel Left Command: " << wl_cmd << endl;
    cout << "Riel Command: " << riel_cmd << endl;
    cout << "Servo Command: " << servo_cmd << endl;

    return 0;
}
