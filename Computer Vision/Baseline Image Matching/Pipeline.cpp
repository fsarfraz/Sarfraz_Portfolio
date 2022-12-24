
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>

#include "Header.h";

using namespace cv;
using namespace std;


/*
  Given a directory on the command line, scans through the directory for image
files.
  Prints out the full path name for each file.  This can be used as an argument to
fopen or to cv::imread.
 */

//sorting tuple vector by the second element acending
bool sortbysec(const tuple<string, float>& a, const tuple<string, float>& b) {
    return(get<1>(a) < get<1>(b));
}

//sorting tuple vector by the second element decending
bool sortbysec2(const tuple<string, float>& a, const tuple<string, float>& b) {
    return(get<1>(a) > get<1>(b));
}

int main(int argc, char* argv[]) {
   char dirname[256];
   char buffer[256];

    FILE* fp;
    DIR* dirp;
    struct dirent* dp;
    int i;
    string a;
    int task = 0;
    //intializing task
    cout << "Please Choose Task # from the following list:" << endl;
    cout << "task1, task2, task3, task4, task5" << endl;
    cout << "Example: 'task1'  will perform Baseline Matching" << endl;
    cin >> a;

  
    // check for sufficient arguments
    if (argc < 2) {
        printf("usage: %s <directory path>\n", argv[0]);
        exit(-1);
    }
    // get the directory path
    strcpy_s(dirname, argv[1]);
    printf("Processing directory %s\n", dirname);

    // open the directory
    dirp = opendir(dirname);
    if (dirp == NULL) {
        printf("Cannot open directory %s\n", dirname);
        exit(-1);
    }
    
    //feature vectors and result vector
    //tuple<string, float> temp;
    vector<tuple<string, float>> result1;
    vector<tuple<string, float>> result2;
    vector<float> target_features;
    vector<float> target_features2;
    vector<float> target_features3;

   //read target image (eventually make a function that takes target image-- finds features and puts into a csv
   char target_file[256];
   char target_file2[256];
   char target_file3[256];
   char target_file4[256];
   char target_file5[256];

   strcpy_s(target_file, "olympus/pic.1016.jpg");
   strcpy_s(target_file2,"olympus/pic.0164.jpg");
   strcpy_s(target_file3,"olympus/pic.0274.jpg");
   strcpy_s(target_file4,"olympus/pic.0535.jpg");
   strcpy_s(target_file5,"olympus/pic.0106.jpg");


   //task 1
   if (a == "task1") {
       baselineMatchingfeatures(target_file, target_features);
   }
  
   //task 2
   else if (a == "task2") {
       histogram(target_file2, target_features);
   }
   
   //task 3
   else if (a == "task3") {
       multiHistogram(target_file3, target_features);
   }

   //task 4
   else if (a == "task4") {
       histogram(target_file4, target_features);
       TextureColor_histogram(target_file4, target_features2);
   }
   
   //task 5
   else if (a == "task5") {
       histogram(target_file5, target_features);
       TextureColor_histogram(target_file5, target_features2);
       HSV_maskHG(target_file5, target_features3);
   }


   // loop over all the files in the image file listing
    while ((dp = readdir(dirp)) != NULL) {
        // check if the file is an image
        if (strstr(dp->d_name, ".jpg") ||
            strstr(dp->d_name, ".png") ||
            strstr(dp->d_name, ".ppm") ||
            strstr(dp->d_name, ".tif")) {

            printf("processing image file: %s\n", dp->d_name);
            // build the overall filename
            strcpy_s(buffer, dirname);
            strcat_s(buffer, "/");
            strcat_s(buffer, dp->d_name);
            
           //read database images and compare
           vector<float> database_features;
           vector<float> database_features2;
           vector<float> database_features3;

          // task 1
           if (a == "task1") {
               baselineMatchingfeatures(buffer, database_features);
               SSD(buffer, target_features, database_features, result2);
           }
           
          //task 2
           else if (a == "task2") {
               histogram(buffer, database_features);
               hg_intersection(buffer, target_features, database_features, result2);
           }

          //task 3
           else if (a == "task3") {
               multiHistogram(buffer, database_features);
               hg_intersection(buffer, target_features, database_features, result2);
           }

          //task 4
           else if (a == "task4") {
               histogram(buffer, database_features);
               TextureColor_histogram(buffer, database_features2);
               TextureColor_metric(buffer, target_features, target_features2, database_features, database_features2, result2); // 4 histograms enter distance metric
           }

          //task5
           else if (a == "task5") {
               histogram(buffer, database_features);
               TextureColor_histogram(buffer, database_features2);
               HSV_maskHG(buffer, database_features3);
               TextureColorHSV_metric(buffer, target_features, target_features2, target_features3, database_features, database_features2, database_features3, result2);
           }

           //printf("full path name: %s\n", buffer);
        }

    }

   //results in vector tuple that prints out filename and distance metric from least to greatest
    sort(result2.begin(), result2.end(), sortbysec);
    for (i = 0; i < 11; i++) {
        cout << "\n" << get<0>(result2[i]) << endl;
        cout << get<1>(result2[i])<< endl;
    }
    //results in vector tuple that prints out filename and distance metric from greatest to least
    sort(result2.begin(), result2.end(), sortbysec2);
    for (i = 0; i < 11; i++) {
        cout << "\n" << get<0>(result2[i]) << endl;
        cout << get<1>(result2[i])<< endl;
    }


    return(0);
}