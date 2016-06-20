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

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.opencv.core.Point3;

public class MarkerConfig {

	static Map<String, Marker> map = null;
	static float size = 0;

	public static List<String> readFromConfig(String configfile) {
		map = new HashMap<>();
		List<String> patternlist = new ArrayList<>();
		try {
			BufferedReader fr = new BufferedReader(new FileReader(configfile));
			String s;
			while ((s = fr.readLine()) != null) {
				if (s.contains("markersize")){
					size = Float.parseFloat(s.split(" ")[1]);
					//System.out.println(s+" "+Float.parseFloat(s.split(" ")[1]));
				}
				if (s.contains("patt")) {
					try {
						String[] values = s.split(" ");
						float x = Float.parseFloat(values[0]);
						float y = Float.parseFloat(values[1]);
						float z = Float.parseFloat(values[2]);// assuming z does NOT
																// change (x-y
																// plane)
						String pattern = values[3];
						Marker m = new Marker(pattern);
						m.upperleft = new Point3(x, y, z);
						m.upperright = new Point3(x + size, y, z);
						m.lowerright = new Point3(x + size, y - size, z);
						m.lowerleft = new Point3(x, y - size, z);
						map.put(pattern, m);
						patternlist.add(pattern);
						//System.out.println("Adding pattern "+pattern);
					} catch (NumberFormatException e) {
						continue;
					}
				}
			}
			fr.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		return patternlist;
	}

	public static void init() {
		map = new HashMap<>();
		// uppper left
		Marker ul = new Marker("4x4_98.patt");
		ul.upperleft = new Point3(0, 0, 0);
		ul.upperright = new Point3(0, -0.135, 0);
		ul.lowerright = new Point3(0, -0.135, -0.135);
		ul.lowerleft = new Point3(0, 0, -0.135);
		map.put(ul.pattern, ul);

		// uppper right
		Marker ur = new Marker("4x4_1.patt");
		ur.upperleft = new Point3(0, -0.98, 0);
		ur.upperright = new Point3(0, -1.115, 0);
		ur.lowerright = new Point3(0, -1.115, -0.135);
		ur.lowerleft = new Point3(0, -0.98, -0.135);
		map.put(ur.pattern, ur);

		// lower right
		Marker lr = new Marker("4x4_2.patt");
		lr.upperleft = new Point3(0, -0.98, -0.49);
		lr.upperright = new Point3(0, -1.115, -0.49);
		lr.lowerright = new Point3(0, -1.115, -0.625);
		lr.lowerleft = new Point3(0, -0.98, -0.625);
		map.put(lr.pattern, lr);

		// lower left
		Marker ll = new Marker("4x4_3.patt");
		ll.upperleft = new Point3(0, 0, -0.49);
		ll.upperright = new Point3(0, -0.135, -0.49);
		ll.lowerright = new Point3(0, -0.135, -0.625);
		ll.lowerleft = new Point3(0, 0, -0.625);
		map.put(ll.pattern, ll);
	}

	public static List<Point3> create3dpointlist(String string) {
		if (map == null)
			init();
		if (map.containsKey(string)) {
			Marker m = map.get(string);
			List<Point3> list = new ArrayList<Point3>();
			list.add(m.upperleft);
			list.add(m.upperright);
			list.add(m.lowerright);
			list.add(m.lowerleft);
			return list;
		} else
			return null;
	}

	public static List<Point3> getUnordered3DPointList() {
		if (map == null)
			init();
		List<Point3> list = new ArrayList<>();
		for (String string : map.keySet()) {
			Marker m = map.get(string);
			list.add(m.upperleft);
			list.add(m.upperright);
			list.add(m.lowerright);
			list.add(m.lowerleft);

		}
		return list;
	}

	public static float getMarkerSize() {
		return size;
	}

}

class Marker {

	public Marker(String string) {
		this.pattern = string;
	}

	String pattern;
	Point3 upperleft;
	Point3 upperright;
	Point3 lowerright;
	Point3 lowerleft;
}
