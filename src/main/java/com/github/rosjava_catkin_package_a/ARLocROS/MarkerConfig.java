/*
 * Copyright (C) 2016 Marvin Ferber.
 * 
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

package com.github.rosjava_catkin_package_a.ARLocROS;

import org.opencv.core.Point3;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class MarkerConfig {

    Map<String, Marker> map = null;
    float size = 0;

    public static MarkerConfig createFromConfig(String configfile, String pATTERN_DIR) {
        MarkerConfig mc = new MarkerConfig();
        Map<String, Marker> map = new HashMap<>();
        float size = 0;
        // List<String> patternlist = new ArrayList<>();
        try {
            BufferedReader fr = new BufferedReader(new FileReader(configfile));
            String s;
            while ((s = fr.readLine()) != null) {
                if (s.contains("markersize")) {
                    size = Float.parseFloat(s.split(" ")[1]);
                    // System.out.println(s+" "+Float.parseFloat(s.split("
                    // ")[1]));
                }
                if (s.contains("patt")) {
                    try {
                        String[] values = s.split(" ");
                        float x = Float.parseFloat(values[0]);
                        float y = Float.parseFloat(values[1]);
                        float z = Float.parseFloat(values[2]);

                        String pattern = pATTERN_DIR + values[3];
                        final Marker marker = Marker.builder()
                                .patternFile(pattern)
                                .upperleft(new Point3(x, y, z))
                                .upperright(new Point3(x + size, y, z))
                                .lowerright(new Point3(x + size, y - size, z))
                                .lowerleft(new Point3(x, y - size, z))
                                .build();

                        map.put(pattern, marker);
                        // patternlist.add(pattern);
                        // System.out.println("Adding pattern "+pattern);
                    } catch (NumberFormatException e) {
                        continue;
                    }
                }
            } fr.close();
        } catch (IOException e) {
            e.printStackTrace();
        } mc.setPatternMap(map);
        mc.setPatternSize(size);
        return mc;
    }

    private void setPatternSize(float size2) {
        this.size = size2;

    }

    private void setPatternMap(Map<String, Marker> map2) {
        this.map = map2;

    }

    public List<Point3> create3dpointlist(String string) {
        if (map != null && map.containsKey(string)) {
            Marker m = map.get(string);
            List<Point3> list = new ArrayList<Point3>();
            list.add(m.upperleft());
            list.add(m.upperright());
            list.add(m.lowerright());
            list.add(m.lowerleft());
            return list;
        } else {
            return null;
        }
    }

    public List<Point3> getUnordered3DPointList() {
        if (map == null) {
            return null;
        }
        List<Point3> list = new ArrayList<>();
        for (String string : map.keySet()) {
            Marker m = map.get(string);
            list.add(m.upperleft());
            list.add(m.upperright());
            list.add(m.lowerright());
            list.add(m.lowerleft());

        }
        return list;
    }

    public float getMarkerSize() {
        return size;
    }

    public List<String> getPatternFileList() {
        List<String> patternlist = new ArrayList<>();
        for (String pattern : map.keySet()) {
            patternlist.add(map.get(pattern).patternFile());
        }
        return patternlist;
    }

}