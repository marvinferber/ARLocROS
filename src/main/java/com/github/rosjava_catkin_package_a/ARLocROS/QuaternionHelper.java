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


public class QuaternionHelper {
	private float x, y, z, w;

	
	
	public QuaternionHelper() {
		x = y = z = 0;
		w = 1;
	}

	public final float getW() {
		return w;
	}

	public final void setW(final float w) {
		this.w = w;
	}

	public final float getX() {
		return x;
	}

	public final void setX(final float x) {
		this.x = x;
	}

	public final float getY() {
		return y;
	}

	public final void setY(final float y) {
		this.y = y;
	}

	public final float getZ() {
		return z;
	}

	public final void setZ(final float z) {
		this.z = z;
	}

	public final QuaternionHelper normalize() {
		final float norm = (float) Math.sqrt(w * w + x * x + y * y + z * z);

		final float invNorm = 1f / norm;
		w *= invNorm;
		x *= invNorm;
		y *= invNorm;
		z *= invNorm;

		return this;
	}

	public final QuaternionHelper setFromEuler(final float bankX, final float headingY, final float attitudeZ) {

		float angle = headingY * 0.5f;
		final float sinHeadingY = (float) Math.sin(angle);
		final float cosHeadingY = (float) Math.cos(angle);
		angle = attitudeZ * 0.5f;
		final float sinAttitudeZ = (float) Math.sin(angle);
		final float cosAttitudeZ = (float) Math.cos(angle);
		angle = bankX * 0.5f;
		final float sinBankX = (float) Math.sin(angle);
		final float cosBankX = (float) Math.cos(angle);

		// variables used to reduce multiplication calls.
		final float cosHeadingXcosAttitude = cosHeadingY * cosAttitudeZ;
		final float sinHeadingXsinAttitude = sinHeadingY * sinAttitudeZ;
		final float cosHeadingXsinAttitude = cosHeadingY * sinAttitudeZ;
		final float sinHeadingXcosAttitude = sinHeadingY * cosAttitudeZ;

		w = cosHeadingXcosAttitude * cosBankX - sinHeadingXsinAttitude * sinBankX;
		x = cosHeadingXcosAttitude * sinBankX + sinHeadingXsinAttitude * cosBankX;
		y = sinHeadingXcosAttitude * cosBankX + cosHeadingXsinAttitude * sinBankX;
		z = cosHeadingXsinAttitude * cosBankX - sinHeadingXcosAttitude * sinBankX;
		return normalize();

	}

	public String toString() {
		return "Quaternion[x " + x + ", y " + y + ", z " + z + ", w " + w + "]";
	}
}
