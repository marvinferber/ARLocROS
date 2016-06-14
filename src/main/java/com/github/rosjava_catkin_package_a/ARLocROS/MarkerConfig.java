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

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Point3;

public class MarkerConfig {

	public static List<Point3> create3dpointlist() {
		List<Point3> list = new ArrayList<Point3>();
		// uppper left
		Point3 p = new Point3(0, 0, 0);
		list.add(p);
		p = new Point3(0, -0.135, 0);
		list.add(p);
		p = new Point3(0, -0.135, -0.135);
		list.add(p);
		p = new Point3(0, 0, -0.135);
		list.add(p);
		// uppper right
		p = new Point3(0, -0.98, 0);
		list.add(p);
		p = new Point3(0, -1.115, 0);
		list.add(p);
		p = new Point3(0, -1.115, -0.135);
		list.add(p);
		p = new Point3(0, -0.98, -0.135);
		list.add(p);
		// lower right
		p = new Point3(0, -0.98, -0.49);
		list.add(p);
		p = new Point3(0, -1.115, -0.49);
		list.add(p);
		p = new Point3(0, -1.115, -0.625);
		list.add(p);
		p = new Point3(0, -0.98, -0.625);
		list.add(p);
		// lower left
		p = new Point3(0, 0, -0.49);
		list.add(p);
		p = new Point3(0, -0.135, -0.49);
		list.add(p);
		p = new Point3(0, -0.135, -0.625);
		list.add(p);
		p = new Point3(0, 0, -0.625);
		list.add(p);
		return list;
	}

}
