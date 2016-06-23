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

import org.ros.message.Time;
import org.ros.rosjava_geometry.Transform;
import org.ros.rosjava_geometry.Vector3;

public class PoseCompare {

	/**
	 * @param current_pose
	 * @param last_pose
	 * @return
	 */
	public static double distance(Transform current_pose, Transform last_pose) {
		Vector3 point1 = current_pose.getTranslation();
		Vector3 point2 = last_pose.getTranslation();
		double distance = Math.sqrt(Math.pow((point1.getX() - point2.getX()), 2)
				+ Math.pow((point1.getY() - point2.getY()), 2) + Math.pow((point1.getZ() - point2.getZ()), 2));
		return distance;
	}

	/**
	 * @param current_timestamp
	 * @param last_timestamp
	 * @return
	 */
	public static double timedelta(Time current_timestamp, Time last_timestamp) {
		long lasttime = last_timestamp.totalNsecs();
		long currenttime = current_timestamp.totalNsecs();
		return (currenttime - lasttime) / 1000000000.0;

	}

}
