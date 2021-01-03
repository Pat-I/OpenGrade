using SharpGL;
using System;
using System.Collections.Generic;

namespace OpenGrade
{
    public class CContourPt
    {
        public double altitude { get; set; }
        public double easting { get; set; }
        public double northing { get; set; }
        public double heading { get; set; }
        public double cutAltitude { get; set; }
        public double lastPassAltitude { get; set; }
        public double latitude { get; set; }
        public double longitude { get; set; }
        public double distance { get; set; }

        //constructor
        public CContourPt(double _easting, double _heading, double _northing,
                            double _altitude, double _lat, double _long,
                            double _cutAltitude = -1, double _lastPassAltitude = -1, double _distance = -1)
        {
            easting = _easting;
            northing = _northing;
            heading = _heading;
            altitude = _altitude;
            latitude = _lat;
            longitude = _long;

            //optional parameters
            cutAltitude = _cutAltitude;
            lastPassAltitude = _lastPassAltitude;
            distance = _distance;
        }
    }

    public class CCurSwathPt
    {
        public double altitude { get; set; }
        public double easting { get; set; }
        public double northing { get; set; }
        public double heading { get; set; }
        public double cutAltitude { get; set; }
        public double realPassAltitude { get; set; }
        public double latitude { get; set; }
        public double longitude { get; set; }
        public int swathNbr { get; set; }

        //constructor
        public CCurSwathPt(double _easting, double _heading, double _northing,
                            double _altitude, double _lat, double _long,
                            double _cutAltitude = -1, double _realPassAltitude = -1, int _swathNbr = 0)
        {
            easting = _easting;
            northing = _northing;
            heading = _heading;
            altitude = _altitude;
            latitude = _lat;
            longitude = _long;

            //optional parameters
            cutAltitude = _cutAltitude;
            realPassAltitude = _realPassAltitude;
            swathNbr = _swathNbr;
        }
    }

    public class CSwathPt
    {
        public double altitude { get; set; }
        public double easting { get; set; }
        public double northing { get; set; }
        public double heading { get; set; }
        public double cutAltitude { get; set; }
        public double realPassAltitude { get; set; }
        public double latitude { get; set; }
        public double longitude { get; set; }
        public int swathNbr { get; set; }

        //constructor
        public CSwathPt(double _easting, double _heading, double _northing,
                            double _altitude, double _lat, double _long,
                            double _cutAltitude = -1, double _realPassAltitude = -1, int _swathNbr = 0)
        {
            easting = _easting;
            northing = _northing;
            heading = _heading;
            altitude = _altitude;
            latitude = _lat;
            longitude = _long;

            //optional parameters
            cutAltitude = _cutAltitude;
            realPassAltitude = _realPassAltitude;
            swathNbr = _swathNbr;
        }
    }

    public class CContour
    {
        //copy of the mainform address
        private readonly FormGPS mf;

        private readonly OpenGL gl;

        public bool isContourOn, isContourBtnOn;
        public double slope = 0.002;
        public double zeroAltitude = 0;
        public double zeroAltitudeRef = 0;

        public List<CContourPt> ptList = new List<CContourPt>();
        public List<CCurSwathPt> curSwathList = new List<CCurSwathPt>();
        public List<CSwathPt> swathList = new List<CSwathPt>();

        //Variables for the swath lists
        public int currentSwathNbr;


        //used to determine if section was off and now is on or vice versa
        public bool wasSectionOn;

        //generated box for finding closest point
        public vec2 boxA = new vec2(0, 0), boxB = new vec2(0, 2);

        public vec2 boxC = new vec2(1, 1), boxD = new vec2(2, 3);

        //angle to path line closest point and fix
        public double refHeading, ref2;

        // for closest line point to current fix
        public double minDistance = 99999.0, refX, refZ;

        //generated reference line
        public double refLineSide = 1.0;

        public vec2 refPoint1 = new vec2(1, 1), refPoint2 = new vec2(2, 2);

        public double distanceFromRefLine;
        public double distanceFromCurrentLine;

        private int A, B, C;
        public double abFixHeadingDelta, abHeading;

        public bool isABSameAsFixHeading = true;
        public bool isOnRightSideCurrentLine = true;

        public bool isDrawingRefLine;

        //pure pursuit values
        public vec2 goalPointCT = new vec2(0, 0);

        public vec2 radiusPointCT = new vec2(0, 0);
        public double steerAngleCT;
        public double rEastCT, rNorthCT;
        public double ppRadiusCT;

        //list of contour data from GPS
        //public List<vec4> ptList = new List<vec4>();

        //the manually picked list
        public List<vec2> drawList = new List<vec2>();

        //converted from drawn line to all points cut line
        //public List<vec4> cutList = new List<vec4>();

        //list of the list of individual Lines for entire field
        //public List<CContourPt> topoList = new List<CContourPt>();

        //constructor
        public CContour(OpenGL _gl, FormGPS _f)
        {
            mf = _f;
            if (mf != null)
            {
                gl = _gl;
            }
        }

        //start stop and add points to list
        public void StartContourLine()
        {
            isContourOn = true;
            //reuse ptList
            ptList.Clear();

            CContourPt point = new CContourPt(mf.pn.easting, mf.fixHeading, mf.pn.northing, mf.pn.altitude, mf.pn.latitude, mf.pn.longitude);
            ptList.Add(point);
        }

        //Add current position to ptList
        public void AddPoint()
        {
            CContourPt point = new CContourPt(mf.pn.easting, mf.fixHeading, mf.pn.northing, mf.pn.altitude, mf.pn.latitude, mf.pn.longitude);
            ptList.Add(point);
        }

        //End the strip
        public void StopContourLine()
        {
            CContourPt point = new CContourPt(mf.pn.easting, mf.fixHeading, mf.pn.northing, mf.pn.altitude, mf.pn.latitude, mf.pn.longitude);
            ptList.Add(point);

            //turn it off
            isContourOn = false;
        }

        // Invert ptList pts
        public void InvertPtList()
        {
            if (ptList.Count > 4)
            {
                //ptList.Reverse(); //No because we have to change the heading
                
                int ptcnt = ptList.Count;
                for(int h = 0; h<= (ptcnt/2 -1); h++)
                {
                    double tempEasting = ptList[h].easting;
                    double tempHeading = ptList[h].heading;
                    double tempNorthing = ptList[h].northing;
                    double tempAltitude = ptList[h].altitude;
                    double tempLatitude = ptList[h].latitude;
                    double tempLongitude = ptList[h].longitude;
                    double tempCutAltitude = ptList[h].cutAltitude;
                    double tempLastPassAltitude = ptList[h].lastPassAltitude;
                    //double tempDistance = ptList[h].distance;

                    tempHeading += Math.PI;
                    if (tempHeading >= glm.twoPI) tempHeading -= glm.twoPI;

                    ptList[h].easting = ptList[ptcnt - 1 - h].easting;
                    double tempSecondHeading = ptList[ptcnt - 1 - h].heading;
                    tempSecondHeading += Math.PI;
                    if (tempSecondHeading >= glm.twoPI) tempSecondHeading -= glm.twoPI;
                    ptList[h].heading = tempSecondHeading;
                    ptList[h].northing = ptList[ptcnt - 1 - h].northing;
                    ptList[h].altitude = ptList[ptcnt - 1 - h].altitude;
                    ptList[h].latitude = ptList[ptcnt - 1 - h].latitude;
                    ptList[h].longitude = ptList[ptcnt - 1 - h].longitude;
                    ptList[h].cutAltitude = ptList[ptcnt - 1 - h].cutAltitude;
                    ptList[h].lastPassAltitude = ptList[ptcnt - 1 - h].lastPassAltitude;
                    //ptList[h].distance = ptList[ptcnt - 1 - h].distance;

                    ptList[ptcnt - 1 - h].easting = tempEasting;
                    ptList[ptcnt - 1 - h].heading = tempHeading;
                    ptList[ptcnt - 1 - h].northing = tempNorthing;
                    ptList[ptcnt - 1 - h].altitude = tempAltitude;
                    ptList[ptcnt - 1 - h].latitude = tempLatitude;
                    ptList[ptcnt - 1 - h].longitude = tempLongitude;
                    ptList[ptcnt - 1 - h].cutAltitude = tempCutAltitude;
                    ptList[ptcnt - 1 - h].lastPassAltitude = tempLastPassAltitude;
                    //ptList[ptcnt - 1 - h].distance = tempDistance;
                }

                // Calculate the distance
                int cnt = ptList.Count;
                if (cnt > 0)
                {
                    ptList[0].distance = 0;
                    for (int i = 0; i < cnt - 1; i++)
                    {
                        ptList[i + 1].distance = mf.pn.Distance(ptList[i].northing, ptList[i].easting, ptList[i + 1].northing, ptList[i + 1].easting);
                    }
                }

            }
        }
        //Save the swath to the main file
        public void SaveSwathToList()
        {
            if (curSwathList.Count > 1)
            {
                int ptcnt = curSwathList.Count;

                //check if swath number fit
                if (swathList.Count > 0)
                {
                    int ptcount = swathList.Count;
                    if (swathList[ptcount - 1].swathNbr != (currentSwathNbr-1))
                    {                       
                        var form = new FormTimedMessage(4000, "Error in swath number", "Report the error");
                        form.Show();

                        
                    }
                }

                for (int h = 0; h < ptcnt; h++)
                {
                    //fill the current swath list point
                    CSwathPt point = new CSwathPt(curSwathList[h].easting, curSwathList[h].heading, curSwathList[h].northing, curSwathList[h].altitude, curSwathList[h].latitude, curSwathList[h].longitude, curSwathList[h].cutAltitude, curSwathList[h].realPassAltitude, currentSwathNbr);
                    swathList.Add(point);
                }

                currentSwathNbr++;
                mf.FileSaveSwath();
            }
            curSwathList.Clear();
        }

        public void ClearSwathList()
        {
            swathList.Clear();
            currentSwathNbr = 0;
        }

        //determine distance from contour guidance line
        public void DistanceFromContourLine()
        {
            double minDistA = 1000000, minDistB = 1000000;
            int ptCount = ptList.Count;
            //distanceFromCurrentLine = 9999;
            if (ptCount > 0)
            {
                //find the closest 2 points to current fix
                for (int t = 0; t < ptCount; t++)
                {
                    double dist = ((mf.pn.easting - ptList[t].easting) * (mf.pn.easting - ptList[t].easting))
                                    + ((mf.pn.northing - ptList[t].northing) * (mf.pn.northing - ptList[t].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        B = A;
                        minDistA = dist;
                        A = t;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        B = t;
                    }
                }

                //just need to make sure the points continue ascending or heading switches all over the place
                if (A > B) { C = A; A = B; B = C; }

                //get the distance from currently active AB line
                //x2-x1
                double dx = ptList[B].easting - ptList[A].easting;
                //z2-z1
                double dz = ptList[B].northing - ptList[A].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return;

                //abHeading = Math.Atan2(dz, dx);
                abHeading = ptList[A].heading;

                //how far from current AB Line is fix
                distanceFromCurrentLine = ((dz * mf.pn.easting) - (dx * mf.pn.northing)
                    + (ptList[B].easting * ptList[A].northing) - (ptList[B].northing * ptList[A].easting))
                                / Math.Sqrt((dz * dz) + (dx * dx));

                //are we on the right side or not
                isOnRightSideCurrentLine = distanceFromCurrentLine > 0;

                //absolute the distance
                distanceFromCurrentLine = Math.Abs(distanceFromCurrentLine);

                // ** Pure pursuit ** - calc point on ABLine closest to current position
                double U = (((mf.pn.easting - ptList[A].easting) * (dx))
                            + ((mf.pn.northing - ptList[A].northing) * (dz)))
                            / ((dx * dx) + (dz * dz));

                rEastCT = ptList[A].easting + (U * (dx));
                rNorthCT = ptList[A].northing + (U * (dz));

                //Subtract the two headings, if > 1.57 its going the opposite heading as refAB
                abFixHeadingDelta = (Math.Abs(mf.fixHeading - abHeading));
                if (abFixHeadingDelta >= Math.PI) abFixHeadingDelta = Math.Abs(abFixHeadingDelta - glm.twoPI);

                //used for accumulating distance to find goal point
                double distSoFar;

                //how far should goal point be away  - speed * seconds * kmph -> m/s + min value
                double goalPointDistance = mf.pn.speed * mf.vehicle.goalPointLookAhead * 0.27777777;

                //minimum of 4.0 meters look ahead
                if (goalPointDistance < 3.0) goalPointDistance = 3.0;

                // used for calculating the length squared of next segment.
                double tempDist = 0.0;

                if (abFixHeadingDelta >= glm.PIBy2)
                {
                    //counting down
                    isABSameAsFixHeading = false;
                    distSoFar = mf.pn.Distance(ptList[A].northing, ptList[A].easting, rNorthCT, rEastCT);
                    //Is this segment long enough to contain the full lookahead distance?
                    if (distSoFar > goalPointDistance)
                    {
                        //treat current segment like an AB Line
                        goalPointCT.easting = rEastCT - (Math.Sin(ptList[A].heading) * goalPointDistance);
                        goalPointCT.northing = rNorthCT - (Math.Cos(ptList[A].heading) * goalPointDistance);
                    }

                    //multiple segments required
                    else
                    {
                        //cycle thru segments and keep adding lengths. check if start and break if so.
                        while (A > 0)
                        {
                            B--; A--;
                            tempDist = mf.pn.Distance(ptList[B].northing, ptList[B].easting, ptList[A].northing, ptList[A].easting);

                            //will we go too far?
                            if ((tempDist + distSoFar) > goalPointDistance)
                            {
                                //A++; B++;
                                break; //tempDist contains the full length of next segment
                            }
                            else
                            {
                                distSoFar += tempDist;
                            }
                        }

                        double t = (goalPointDistance - distSoFar); // the remainder to yet travel
                        t /= tempDist;

                        goalPointCT.easting = (((1 - t) * ptList[B].easting) + (t * ptList[A].easting));
                        goalPointCT.northing = (((1 - t) * ptList[B].northing) + (t * ptList[A].northing));
                    }
                }
                else
                {
                    //counting up
                    isABSameAsFixHeading = true;
                    distSoFar = mf.pn.Distance(ptList[B].northing, ptList[B].easting, rNorthCT, rEastCT);

                    //Is this segment long enough to contain the full lookahead distance?
                    if (distSoFar > goalPointDistance)
                    {
                        //treat current segment like an AB Line
                        goalPointCT.easting = rEastCT + (Math.Sin(ptList[A].heading) * goalPointDistance);
                        goalPointCT.northing = rNorthCT + (Math.Cos(ptList[A].heading) * goalPointDistance);
                    }

                    //multiple segments required
                    else
                    {
                        //cycle thru segments and keep adding lengths. check if end and break if so.
                        // ReSharper disable once LoopVariableIsNeverChangedInsideLoop
                        while (B < ptCount - 1)
                        {
                            B++; A++;
                            tempDist = mf.pn.Distance(ptList[B].northing, ptList[B].easting, ptList[A].northing, ptList[A].easting);

                            //will we go too far?
                            if ((tempDist + distSoFar) > goalPointDistance)
                            {
                                //A--; B--;
                                break; //tempDist contains the full length of next segment
                            }

                            distSoFar += tempDist;
                        }

                        //xt = (((1 - t) * x0 + t * x1)
                        //yt = ((1 - t) * y0 + t * y1))

                        double t = (goalPointDistance - distSoFar); // the remainder to yet travel
                        t /= tempDist;

                        goalPointCT.easting = (((1 - t) * ptList[A].easting) + (t * ptList[B].easting));
                        goalPointCT.northing = (((1 - t) * ptList[A].northing) + (t * ptList[B].northing));
                    }
                }

                //calc "D" the distance from pivot axle to lookahead point
                double goalPointDistanceSquared = mf.pn.DistanceSquared(goalPointCT.northing, goalPointCT.easting, mf.pn.northing, mf.pn.easting);

                //calculate the the delta x in local coordinates and steering angle degrees based on wheelbase
                double localHeading = glm.twoPI - mf.fixHeading;
                ppRadiusCT = goalPointDistanceSquared / (2 * (((goalPointCT.easting - mf.pn.easting) * Math.Cos(localHeading)) + ((goalPointCT.northing - mf.pn.northing) * Math.Sin(localHeading))));

                steerAngleCT = glm.toDegrees(Math.Atan(2 * (((goalPointCT.easting - mf.pn.easting) * Math.Cos(localHeading))
                    + ((goalPointCT.northing - mf.pn.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

                if (steerAngleCT < -mf.vehicle.maxSteerAngle) steerAngleCT = -mf.vehicle.maxSteerAngle;
                if (steerAngleCT > mf.vehicle.maxSteerAngle) steerAngleCT = mf.vehicle.maxSteerAngle;

                if (ppRadiusCT < -500) ppRadiusCT = -500;
                if (ppRadiusCT > 500) ppRadiusCT = 500;

                radiusPointCT.easting = mf.pn.easting + (ppRadiusCT * Math.Cos(localHeading));
                radiusPointCT.northing = mf.pn.northing + (ppRadiusCT * Math.Sin(localHeading));

                //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
                double angVel = glm.twoPI * 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(steerAngleCT))) / mf.vehicle.wheelbase;

                //clamp the steering angle to not exceed safe angular velocity
                if (Math.Abs(angVel) > mf.vehicle.maxAngularVelocity)
                {
                    steerAngleCT = glm.toDegrees(steerAngleCT > 0 ?
                            (Math.Atan((mf.vehicle.wheelbase * mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.pn.speed * 0.277777)))
                        : (Math.Atan((mf.vehicle.wheelbase * -mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.pn.speed * 0.277777))));
                }
                //Convert to centimeters
                distanceFromCurrentLine = Math.Round(distanceFromCurrentLine * 1000.0, MidpointRounding.AwayFromZero);

                //distance is negative if on left, positive if on right
                //if you're going the opposite direction left is right and right is left
                //double temp;
                if (isABSameAsFixHeading)
                {
                    //temp = (abHeading);
                    if (!isOnRightSideCurrentLine) distanceFromCurrentLine *= -1.0;
                }

                //opposite way so right is left
                else
                {
                    //temp = (abHeading - Math.PI);
                    //if (temp < 0) temp = (temp + glm.twoPI);
                    //temp = glm.toDegrees(temp);
                    if (isOnRightSideCurrentLine) distanceFromCurrentLine *= -1.0;
                }

                mf.guidanceLineDistanceOff = (Int16)distanceFromCurrentLine;
                mf.guidanceLineSteerAngle = (Int16)(steerAngleCT * 10);
                //mf.guidanceLineHeadingDelta = (Int16)((Math.Atan2(Math.Sin(temp - mf.fixHeading),
                //                                    Math.Cos(temp - mf.fixHeading))) * 10000);
            }
            else
            {
                //invalid distance so tell AS module
                distanceFromCurrentLine = 32000;
                mf.guidanceLineDistanceOff = 32000;
            }
        }

        //draw the red follow me line
        public void DrawContourLine()
        {
            //gl.Color(0.98f, 0.98f, 0.50f);
            //gl.Begin(OpenGL.GL_LINE_STRIP);
            ////for (int h = 0; h < ptCount; h++) gl.Vertex(guideList[h].x, 0, guideList[h].z);
            //gl.Vertex(boxA.easting, boxA.northing, 0);
            //gl.Vertex(boxB.easting, boxB.northing, 0);
            //gl.Vertex(boxC.easting, boxC.northing, 0);
            //gl.Vertex(boxD.easting, boxD.northing, 0);
            //gl.Vertex(boxA.easting, boxA.northing, 0);
            //gl.End();

            //Draw the swath
            int curSwathCnt = curSwathList.Count;
            if (curSwathCnt > 0 & mf.isJobStarted)
            {
                gl.LineWidth(2);
                gl.Color(0.1f, 0.95f, 0.05f);
                gl.Begin(OpenGL.GL_LINE_STRIP);
                for (int h = 0; h < curSwathCnt; h++) gl.Vertex(curSwathList[h].easting, curSwathList[h].northing, 0);
                gl.End();
            }

            //Draw the swath
            int SwathCnt = swathList.Count;
            if (SwathCnt > 0 & mf.isJobStarted)
            {
                gl.LineWidth(2);
                gl.Color(0.9f, 0.8f, 0.4f);
                gl.Begin(OpenGL.GL_LINE_STRIP);
                int swathNbr = 0;
                for (int h = 0; h < SwathCnt; h++)
                {
                    if (swathNbr < swathList[h].swathNbr)
                    {
                        gl.End();
                        gl.Begin(OpenGL.GL_LINE_STRIP);
                        swathNbr = swathList[h].swathNbr;
                    }
                    
                    gl.Vertex(swathList[h].easting, swathList[h].northing, 0);
                    
                }
                gl.End();
            }

            ////draw the guidance line
            int ptCount = ptList.Count;
            gl.LineWidth(2);
            gl.Color(0.98f, 0.2f, 0.0f);
            gl.Begin(OpenGL.GL_LINE_STRIP);
            for (int h = 0; h < ptCount; h++) gl.Vertex(ptList[h].easting, ptList[h].northing, 0);
            gl.End();

            gl.PointSize(4.0f);
            gl.Begin(OpenGL.GL_POINTS);

            gl.Color(0.97f, 0.42f, 0.45f);
            for (int h = 0; h < ptCount; h++) gl.Vertex(ptList[h].easting, ptList[h].northing, 0);

            gl.End();
            gl.PointSize(1.0f);

            //draw the reference line
            gl.PointSize(3.0f);
            //if (isContourBtnOn)
            {
                ptCount = ptList.Count;
                if (ptCount > 0)
                {
                    gl.Begin(OpenGL.GL_POINTS);
                    for (int i = 0; i < ptCount; i++)
                    {
                        gl.Vertex(ptList[i].easting, ptList[i].northing, 0);
                    }
                    gl.End();
                }
            }

            //ptCount = conList.Count;
            //if (ptCount > 0)
            //{
            ////draw closest point and side of line points
            //gl.Color(0.5f, 0.900f, 0.90f);
            //gl.PointSize(4.0f);
            //gl.Begin(OpenGL.GL_POINTS);
            //for (int i = 0; i < ptCount; i++)  gl.Vertex(conList[i].x, conList[i].z, 0);
            //gl.End();

            //gl.Color(0.35f, 0.30f, 0.90f);
            //gl.PointSize(6.0f);
            //gl.Begin(OpenGL.GL_POINTS);
            //gl.Vertex(conList[closestRefPoint].x, conList[closestRefPoint].z, 0);
            //gl.End();
            //}
            if (mf.isPureDisplayOn)
            {
                const int numSegments = 100;
                {
                    gl.Color(0.95f, 0.30f, 0.950f);

                    double theta = glm.twoPI / (numSegments);
                    double c = Math.Cos(theta);//precalculate the sine and cosine
                    double s = Math.Sin(theta);

                    double x = ppRadiusCT;//we start at angle = 0
                    double y = 0;

                    gl.LineWidth(1);
                    gl.Begin(OpenGL.GL_LINE_LOOP);
                    for (int ii = 0; ii < numSegments; ii++)
                    {
                        //glVertex2f(x + cx, y + cy);//output vertex
                        gl.Vertex(x + radiusPointCT.easting, y + radiusPointCT.northing);//output vertex

                        //apply the rotation matrix
                        double t = x;
                        x = (c * x) - (s * y);
                        y = (s * t) + (c * y);
                    }
                    gl.End();

                    //Draw lookahead Point
                    gl.PointSize(4.0f);
                    gl.Begin(OpenGL.GL_POINTS);

                    //gl.Color(1.0f, 1.0f, 0.25f);
                    //gl.Vertex(rEast, rNorth, 0.0);

                    gl.Color(1.0f, 0.5f, 0.95f);
                    gl.Vertex(goalPointCT.easting, goalPointCT.northing, 0.0);

                    gl.End();
                    gl.PointSize(1.0f);
                }
            }
        }

        //Reset the contour to zip
        public void ResetContour()
        {
            if (ptList != null) ptList.Clear();
        }
    }//class
}//namespace