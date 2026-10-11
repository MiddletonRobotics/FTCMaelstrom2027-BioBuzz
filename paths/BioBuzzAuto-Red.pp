{
    "startPoint":  {
                       "x":  61,
                       "y":  9,
                       "headingDeg":  90,
                       "locked":  false
                   },
    "lines":  [
                  {
                      "kind":  "atomic",
                      "id":  "line1",
                      "name":  "1 Start -\u003e HIVE, shoot preloads",
                      "endPoint":  {
                                       "x":  61,
                                       "y":  34
                                   },
                      "controlPoints":  [

                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  90,
                                      "endDeg":  90
                                  },
                      "color":  "#ffc516",
                      "locked":  false,
                      "waitBeforeMs":  0,
                      "waitAfterMs":  2000,
                      "waitBeforeName":  "",
                      "waitAfterName":  "Shoot"
                  },
                  {
                      "kind":  "atomic",
                      "id":  "line2",
                      "name":  "2 -\u003e Side FLOWER",
                      "endPoint":  {
                                       "x":  16,
                                       "y":  48
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  43,
                                                "y":  37
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  90,
                                      "endDeg":  180
                                  },
                      "color":  "#4fc3f7",
                      "locked":  false,
                      "waitBeforeMs":  0,
                      "waitAfterMs":  1000,
                      "waitBeforeName":  "",
                      "waitAfterName":  "Intake"
                  },
                  {
                      "kind":  "atomic",
                      "id":  "line3",
                      "name":  "3 Side FLOWER -\u003e HIVE, shoot",
                      "endPoint":  {
                                       "x":  61,
                                       "y":  34
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  43,
                                                "y":  37
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  180,
                                      "endDeg":  90
                                  },
                      "color":  "#81c784",
                      "locked":  false,
                      "waitBeforeMs":  0,
                      "waitAfterMs":  2000,
                      "waitBeforeName":  "",
                      "waitAfterName":  "Shoot"
                  },
                  {
                      "kind":  "atomic",
                      "id":  "line4",
                      "name":  "4 -\u003e Far FLOWER (around HIVE)",
                      "endPoint":  {
                                       "x":  48,
                                       "y":  128
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  10,
                                                "y":  25
                                            },
                                            {
                                                "x":  30,
                                                "y":  110
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  90,
                                      "endDeg":  90
                                  },
                      "color":  "#ba68c8",
                      "locked":  false,
                      "waitBeforeMs":  0,
                      "waitAfterMs":  1000,
                      "waitBeforeName":  "",
                      "waitAfterName":  "Intake"
                  },
                  {
                      "kind":  "atomic",
                      "id":  "line5",
                      "name":  "5 Partial PARK (corner in LOADING ZONE)",
                      "endPoint":  {
                                       "x":  14,
                                       "y":  93
                                   },
                      "controlPoints":  [

                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  90,
                                      "endDeg":  90
                                  },
                      "color":  "#e57373",
                      "locked":  false,
                      "waitBeforeMs":  0,
                      "waitAfterMs":  0,
                      "waitBeforeName":  "",
                      "waitAfterName":  ""
                  }
              ],
    "shapes":  [

               ],
    "version":  "1.5.0"
}