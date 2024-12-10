#include "inverseKinematics.h"
#define M_PI 3.1415926535897932384626433832795

void inverseKinematics(float targets[3], float angles[5]) {
    if (sqrt(pow(targets[0], 2) + pow(targets[1], 2) + pow(targets[2], 2)) > 3 + 13 + 12.5) {
        for (int i = 0; i < 5; i++) angles[i] = 6969; // Default angle when invalid
        return;
    }

    float theta1, alpha1, alpha2;

    if (targets[2] == 0)
        alpha1 = M_PI / 2;
    else {
        if (targets[1] <= 0 && targets[2] > 0) alpha1 = atan(abs(targets[1] / targets[2])); // 4th
        else if (targets[1] > 0 && targets[2] > 0) alpha1 = -atan(abs(targets[1] / targets[2])); // 1st
        else if (targets[1] <= 0 && targets[2] < 0) alpha1 = M_PI - atan(abs(targets[1] / targets[2])); // 3rd
        else alpha1 = -1 * (M_PI - atan(abs(targets[1] / targets[2])));
    }

    alpha2 = asin(3 / sqrt(pow(targets[1], 2) + pow(targets[2], 2)));
    theta1 = M_PI / 2 - alpha1 - alpha2;
    angles[0] = theta1;

    float Y2 = cos(theta1) * targets[1] - sin(theta1) * targets[2];

    if (sqrt(pow(targets[0], 2) + pow(Y2, 2)) > 13 + 12.5) {
        for (int i = 0; i < 5; i++) angles[i] = 6969; // Default angle when invalid
        return;
    }

    float theta2 = acos((pow(targets[0], 2) + pow(Y2, 2) - pow(13, 2) - pow(12.5, 2)) / (2 * 13 * 12.5));
    angles[2] = abs(theta2);
    angles[4] = -angles[2];

    float alpha, thetaPlusAlpha;

    if (theta2 == 0)
        alpha = 0;
    else
        alpha = atan((12.5 * sin(angles[2])) / (13 + 12.5 * cos(angles[2])));

    if (targets[0] == 0) {
        if (Y2 <= 0) thetaPlusAlpha = M_PI / 2;
        else thetaPlusAlpha = -M_PI / 2;
    } else {
        if (targets[0] > 0 && Y2 <= 0) thetaPlusAlpha = atan(abs(Y2 / targets[0])); // 4th
        else if (targets[0] > 0 && Y2 > 0) thetaPlusAlpha = -atan(abs(Y2 / targets[0])); // 1st
        else if (targets[0] < 0 && Y2 <= 0) thetaPlusAlpha = M_PI - atan(abs(Y2 / targets[0])); // 3rd
        else thetaPlusAlpha = atan(abs(Y2 / targets[0])) - M_PI;
    }

    angles[1] = thetaPlusAlpha - alpha;
    angles[3] = thetaPlusAlpha + alpha;

    for (int i = 0; i < 5; i++) angles[i] = angles[i] * 180 / M_PI;
    return;
}
