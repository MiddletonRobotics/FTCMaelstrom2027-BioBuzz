{
    "startPoint":  {
                       "x":  83,
                       "y":  135,
                       "headingDeg":  270,
                       "locked":  false
                   },
    "lines":  [
                  {
                      "kind":  "atomic",
                      "id":  "line1",
                      "name":  "1 Start -\u003e HIVE, shoot preloads",
                      "endPoint":  {
                                       "x":  83,
                                       "y":  110
                                   },
                      "controlPoints":  [

                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  270,
                                      "endDeg":  270
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
                                       "x":  128,
                                       "y":  96
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  101,
                                                "y":  107
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  270,
                                      "endDeg":  360
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
                                       "x":  83,
                                       "y":  110
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  101,
                                                "y":  107
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  360,
                                      "endDeg":  270
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
                                       "x":  96,
                                       "y":  16
                                   },
                      "controlPoints":  [
                                            {
                                                "x":  134,
                                                "y":  119
                                            },
                                            {
                                                "x":  114,
                                                "y":  34
                                            }
                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  270,
                                      "endDeg":  270
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
                                       "x":  130,
                                       "y":  51
                                   },
                      "controlPoints":  [

                                        ],
                      "heading":  {
                                      "type":  "linear",
                                      "startDeg":  270,
                                      "endDeg":  270
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