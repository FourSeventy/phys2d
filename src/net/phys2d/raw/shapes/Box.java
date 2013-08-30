/*
 * Phys2D - a 2D physics engine based on the work of Erin Catto.
 * 
 * This source is provided under the terms of the BSD License.
 * 
 * Copyright (c) 2006, Phys2D
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or 
 * without modification, are permitted provided that the following 
 * conditions are met:
 * 
 *  * Redistributions of source code must retain the above 
 *    copyright notice, this list of conditions and the 
 *    following disclaimer.
 *  * Redistributions in binary form must reproduce the above 
 *    copyright notice, this list of conditions and the following 
 *    disclaimer in the documentation and/or other materials provided 
 *    with the distribution.
 *  * Neither the name of the Phys2D/New Dawn Software nor the names of 
 *    its contributors may be used to endorse or promote products 
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS 
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, 
 * OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 */
package net.phys2d.raw.shapes;

import java.util.ArrayList;
import net.phys2d.math.MathUtil;
import net.phys2d.math.Matrix2f;
import net.phys2d.math.ROVector2f;
import net.phys2d.math.Vector2f;

/**
 * A simple box in the engine - defined by a width and height
 * 
 * @author Kevin Glass
 */
public strictfp class Box extends AbstractShape implements DynamicShape {
	/** The size of the box */
	private Vector2f size = new Vector2f();
        
	/**
	 * Create a box in the simulation 
	 * 
	 * @param width The width of a box
	 * @param height The hieght of the box
	 */
	public Box(float width, float height) {
		super();
		
		size.set(width,height);
		bounds = new AABox(size.length(), size.length());
	}

	/**
	 * Get the size of this box
	 * 
	 * @return The size of this box
	 */
	public ROVector2f getSize() {
		return size;
	}

	/**
	 * @see net.phys2d.raw.shapes.Shape#getSurfaceFactor()
	 */
	public float getSurfaceFactor() {
		float x = size.getX();
		float y = size.getY();
		
		return (x * x + y * y);
	}

	/**
	 * Get the current positon of a set of points
	 * 
	 * @param pos The centre of the box
	 * @param rotation The rotation of the box
	 * @return The points building up a box at this position and rotation
	 */
	public Vector2f[] getPoints(ROVector2f pos, float rotation) {
		Matrix2f R = new Matrix2f(rotation);
		Vector2f[] pts = new Vector2f[4];
		ROVector2f h = MathUtil.scale(getSize(),0.5f);

		pts[0] = MathUtil.mul(R, new Vector2f(-h.getX(), -h.getY()));
		pts[0].add(pos);
		pts[1] = MathUtil.mul(R, new Vector2f(h.getX(), -h.getY()));
		pts[1].add(pos);
		pts[2] = MathUtil.mul(R, new Vector2f( h.getX(),  h.getY()));
		pts[2].add(pos);
		pts[3] = MathUtil.mul(R, new Vector2f(-h.getX(),  h.getY()));
		pts[3].add(pos);

		return pts;
	}
        
        /**
        * Checks to see if this shape contains the provided point
        *
        * @param p Point to check
        * @param displacement the position of this shape
        * @param rotation the rotation of this shape
        * @return True if the point is contained within the shape
        */
        public boolean contains(Vector2f p, ROVector2f displacement, float rotation)
        {
            Vector2f[] correctedVertices = this.getPoints(displacement, rotation);
            ArrayList<Line> edges = new ArrayList();

            //Save these for our rotated bounding box
            float xMin, yMin, xMax, yMax;
            xMin = correctedVertices[0].x;
            yMin = correctedVertices[0].y;
            xMax = correctedVertices[0].x;
            yMax = correctedVertices[0].y;

            int l = correctedVertices.length;
            for (int i = 0; i < correctedVertices.length; i++)
            {
                Vector2f vec1 = correctedVertices[i];
                Vector2f vec2 = correctedVertices[(i + 1) % l];

                xMin = Math.min(xMin, vec1.x);
                xMax = Math.max(xMax, vec1.x);
                yMin = Math.min(yMin, vec1.y);
                yMax = Math.max(yMax, vec1.y);

                Line line = new Line(vec1, vec2);
                edges.add(line);
            }

            //Definitely not within the polygon
            if (p.x < xMin || p.x > xMax || p.y < yMin || p.y > yMax)
            {

                return false;
            }
            else
            {
                boolean oddNumberOfLines = false;

                //Create the line we want to test with.
                float epsilon = (xMax - xMin) / 100;
                Line ray = new Line(new Vector2f(xMin - epsilon, p.y), p);

                for (Line edge : edges)
                {
                    if (!(edge.getStart().getY() > p.y && edge.getEnd().getY() > p.y) && !(edge.getStart().getY() < p.y && edge.getEnd().getY() < p.y))
                    {
                        Vector2f intersection = ray.intersect(edge);
                        if (intersection != null)
                        {
                            if (intersection.x >= ray.getStart().getX() && intersection.x <= ray.getEnd().getX())
                            {
                                oddNumberOfLines = !oddNumberOfLines;
                            }
                        }
                    }
                }

                if (oddNumberOfLines)
                    return true;
            }

            return false;
        }
}
