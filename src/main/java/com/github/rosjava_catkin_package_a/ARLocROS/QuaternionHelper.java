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

/**
 * Quaternion class with useful helper functions to convert from Euler angles.
 *
 */
public class QuaternionHelper {
	private double x, y, z, w;

	public QuaternionHelper() {
		x = y = z = 0;
		w = 1.0;
	}

	public final double getW() {
		return w;
	}

	public final void setW(final double w) {
		this.w = w;
	}

	public final double getX() {
		return x;
	}

	public final void setX(final double x) {
		this.x = x;
	}

	public final double getY() {
		return y;
	}

	public final void setY(final double y) {
		this.y = y;
	}

	public final double getZ() {
		return z;
	}

	public final void setZ(final double z) {
		this.z = z;
	}

	public final QuaternionHelper normalize() {
		final double norm = Math.sqrt(w * w + x * x + y * y + z * z);

		final double invNorm = 1.0 / norm;
		w *= invNorm;
		x *= invNorm;
		y *= invNorm;
		z *= invNorm;

		return this;
	}

	/**
	 * @param bankX
	 * @param headingY
	 * @param attitudeZ
	 * @return
	 */
	public final QuaternionHelper setFromEuler(final double bankX, final double headingY, final double attitudeZ) {

		double angle = headingY * 0.5;
		final double sinHeadingY = Math.sin(angle);
		final double cosHeadingY = Math.cos(angle);
		angle = attitudeZ * 0.5;
		final double sinAttitudeZ = Math.sin(angle);
		final double cosAttitudeZ = Math.cos(angle);
		angle = bankX * 0.5;
		final double sinBankX = Math.sin(angle);
		final double cosBankX = Math.cos(angle);

		w = (cosHeadingY * cosAttitudeZ) * cosBankX - (sinHeadingY * sinAttitudeZ) * sinBankX;
		x = (cosHeadingY * cosAttitudeZ) * sinBankX + (sinHeadingY * sinAttitudeZ) * cosBankX;
		y = (sinHeadingY * cosAttitudeZ) * cosBankX + (cosHeadingY * sinAttitudeZ) * sinBankX;
		z = (cosHeadingY * sinAttitudeZ) * cosBankX - (sinHeadingY * cosAttitudeZ) * sinBankX;
		return normalize();

	}

	public String toString() {
		return "Quaternion[x " + x + ", y " + y + ", z " + z + ", w " + w + "]";
	}
}
