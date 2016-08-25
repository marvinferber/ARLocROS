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

import java.awt.Dimension;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;

import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JLabel;

import javax.swing.WindowConstants;

import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/**
 * Helper class to display an OpenCV image represented as a Mat using only one
 * function call.
 *
 */
public class Imshow {

	private static Imshow frame;
	public JFrame Window;
	private ImageIcon image;
	private JLabel label;

	private Boolean SizeCustom;
	private int Height, Width;
	

	public Imshow(String title, int height, int width) {
		SizeCustom = true;
		Height = height;
		Width = width;

		Window = new JFrame();
		image = new ImageIcon();
		label = new JLabel();

		label.setIcon(image);
		Window.getContentPane().add(label);
		Window.setResizable(false);
		Window.setTitle(title);

	}

	/**
	 * @param opencvImage
	 */
	public static void show(Mat opencvImage) {
		
		Dimension frameSize = new Dimension(opencvImage.rows(), opencvImage.cols());
		if (frame == null) {
			frame = new Imshow("", frameSize.height, frameSize.width);
			frame.Window.setResizable(false);

			frame.Window.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
			if (frame.SizeCustom) {
				Imgproc.resize(opencvImage, opencvImage, new Size(frame.Height, frame.Width));
			}
		}
		BufferedImage bufImage = null;
		try {

			int type = BufferedImage.TYPE_BYTE_GRAY;
			if (opencvImage.channels() > 1) {
				type = BufferedImage.TYPE_3BYTE_BGR;
			}
			int bufferSize = opencvImage.channels() * opencvImage.cols() * opencvImage.rows();
			byte[] b = new byte[bufferSize];
			opencvImage.get(0, 0, b);
			BufferedImage bufferedImage = new BufferedImage(opencvImage.cols(), opencvImage.rows(), type);
			final byte[] targetPixels = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
			System.arraycopy(b, 0, targetPixels, 0, b.length);
			bufImage = bufferedImage;
			frame.image.setImage(bufImage);
			frame.Window.pack();
			frame.label.updateUI();
			frame.Window.setVisible(true);
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}
