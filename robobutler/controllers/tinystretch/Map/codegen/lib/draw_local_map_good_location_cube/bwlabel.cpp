//
// File: bwlabel.cpp
//
// MATLAB Coder version            : 5.0
// C/C++ source code generated on  : 27-Sep-2022 17:30:11
//

// Include Files
#include "bwlabel.h"
#include "draw_local_map_good_location_cube.h"

// Function Definitions

//
// Arguments    : const double varargin_1[6400]
//                double L[6400]
//                double *numComponents
// Return Type  : void
//
void bwlabel(const double varargin_1[6400], double L[6400], double
             *numComponents)
{
  int firstRunOnPreviousColumn;
  int numRuns;
  boolean_T im[6400];
  int lastRunOnPreviousColumn;
  coder::array<signed char, 1U> startRow;
  coder::array<signed char, 1U> endRow;
  int k;
  coder::array<signed char, 1U> startCol;
  coder::array<int, 1U> labelForEachRun;
  coder::array<int, 1U> labelsRenumbered;
  *numComponents = 0.0;
  for (firstRunOnPreviousColumn = 0; firstRunOnPreviousColumn < 6400;
       firstRunOnPreviousColumn++) {
    im[firstRunOnPreviousColumn] = (varargin_1[firstRunOnPreviousColumn] != 0.0);
    L[firstRunOnPreviousColumn] = 0.0;
  }

  numRuns = 0;
  for (lastRunOnPreviousColumn = 0; lastRunOnPreviousColumn < 80;
       lastRunOnPreviousColumn++) {
    if (im[80 * lastRunOnPreviousColumn]) {
      numRuns++;
    }

    for (k = 0; k < 79; k++) {
      firstRunOnPreviousColumn = k + 80 * lastRunOnPreviousColumn;
      if (im[firstRunOnPreviousColumn + 1] && (!im[firstRunOnPreviousColumn])) {
        numRuns++;
      }
    }
  }

  if (numRuns != 0) {
    int runCounter;
    int row;
    int firstRunOnThisColumn;
    startRow.set_size(numRuns);
    endRow.set_size(numRuns);
    startCol.set_size(numRuns);
    runCounter = 0;
    for (lastRunOnPreviousColumn = 0; lastRunOnPreviousColumn < 80;
         lastRunOnPreviousColumn++) {
      row = 1;
      while (row <= 80) {
        while ((row <= 80) && (!im[(row + 80 * lastRunOnPreviousColumn) - 1])) {
          row++;
        }

        if ((row <= 80) && im[(row + 80 * lastRunOnPreviousColumn) - 1]) {
          startCol[runCounter] = static_cast<signed char>
            (lastRunOnPreviousColumn + 1);
          startRow[runCounter] = static_cast<signed char>(row);
          while ((row <= 80) && im[(row + 80 * lastRunOnPreviousColumn) - 1]) {
            row++;
          }

          endRow[runCounter] = static_cast<signed char>(row - 1);
          runCounter++;
        }
      }
    }

    labelForEachRun.set_size(numRuns);
    for (firstRunOnPreviousColumn = 0; firstRunOnPreviousColumn < numRuns;
         firstRunOnPreviousColumn++) {
      labelForEachRun[firstRunOnPreviousColumn] = 0;
    }

    k = 0;
    runCounter = 1;
    row = 1;
    firstRunOnPreviousColumn = -1;
    lastRunOnPreviousColumn = -1;
    firstRunOnThisColumn = 0;
    while (k + 1 <= numRuns) {
      if (startCol[k] == runCounter + 1) {
        firstRunOnPreviousColumn = firstRunOnThisColumn + 1;
        firstRunOnThisColumn = k;
        lastRunOnPreviousColumn = k;
        runCounter = startCol[k];
      } else {
        if (startCol[k] > runCounter + 1) {
          firstRunOnPreviousColumn = -1;
          lastRunOnPreviousColumn = -1;
          firstRunOnThisColumn = k;
          runCounter = startCol[k];
        }
      }

      if (firstRunOnPreviousColumn >= 0) {
        for (int p = firstRunOnPreviousColumn - 1; p < lastRunOnPreviousColumn;
             p++) {
          if ((endRow[k] >= startRow[p]) && (startRow[k] <= endRow[p])) {
            if (labelForEachRun[k] == 0) {
              labelForEachRun[k] = labelForEachRun[p];
              row++;
            } else {
              if (labelForEachRun[k] != labelForEachRun[p]) {
                int root_k;
                int root_p;
                for (root_k = k; root_k + 1 != labelForEachRun[root_k]; root_k =
                     labelForEachRun[root_k] - 1) {
                  labelForEachRun[root_k] =
                    labelForEachRun[labelForEachRun[root_k] - 1];
                }

                for (root_p = p; root_p + 1 != labelForEachRun[root_p]; root_p =
                     labelForEachRun[root_p] - 1) {
                  labelForEachRun[root_p] =
                    labelForEachRun[labelForEachRun[root_p] - 1];
                }

                if (root_k + 1 != root_p + 1) {
                  if (root_p + 1 < root_k + 1) {
                    labelForEachRun[root_k] = root_p + 1;
                    labelForEachRun[k] = root_p + 1;
                  } else {
                    labelForEachRun[root_p] = root_k + 1;
                    labelForEachRun[p] = root_k + 1;
                  }
                }
              }
            }
          }
        }
      }

      if (labelForEachRun[k] == 0) {
        labelForEachRun[k] = row;
        row++;
      }

      k++;
    }

    labelsRenumbered.set_size(labelForEachRun.size(0));
    for (k = 0; k < numRuns; k++) {
      if (labelForEachRun[k] == k + 1) {
        (*numComponents)++;
        labelsRenumbered[k] = static_cast<int>(*numComponents);
      }

      labelsRenumbered[k] = labelsRenumbered[labelForEachRun[k] - 1];
      firstRunOnPreviousColumn = startRow[k];
      runCounter = endRow[k];
      for (row = firstRunOnPreviousColumn; row <= runCounter; row++) {
        L[(row + 80 * (startCol[k] - 1)) - 1] = labelsRenumbered[k];
      }
    }
  }
}

//
// File trailer for bwlabel.cpp
//
// [EOF]
//
