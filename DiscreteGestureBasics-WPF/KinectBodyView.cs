//------------------------------------------------------------------------------
// <copyright file="KinectBodyView.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DiscreteGestureBasics
{
    using System;
    using System.Collections.Generic;
    using System.Windows;
    using System.Windows.Media;
    using Microsoft.Kinect;
    using System.Diagnostics;
    using System.Timers;


    /// <summary>
    /// Visualizes the Kinect Body stream for display in the UI
    /// </summary>
    public sealed class KinectBodyView : Window
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as closed
        /// </summary>
        private readonly Brush handClosedBrush = new SolidColorBrush(Color.FromArgb(128, 255, 0, 0));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as opened
        /// </summary>
        private readonly Brush handOpenBrush = new SolidColorBrush(Color.FromArgb(40, 151, 191, 214));

        /// <summary>
        /// Brush used for drawing hands that are currently tracked as in lasso (pointer) position
        /// </summary>
        private readonly Brush handLassoBrush = new SolidColorBrush(Color.FromArgb(128, 0, 0, 255));

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        Timer timer = new Timer();
        Timer timerCancel = new Timer();
        static int compteur = 0;
        static int compteurCancel = 0;
        string positionMain = "";
        bool success = false;
        bool successVideo = false;
        static string mainActive = "left";
        int scene = 0;
        int compteurVideo;
        Timer timerVideo = new Timer();
        int compteurIntro;
        Timer timerIntro = new Timer();
        bool videoStarted = false;

        int compteurFinIntro;
        Timer timerFinIntro = new Timer();
        bool accederMain = false;

        int compteurCancelFinIntro;
        Timer timerCancelFinIntro = new Timer();

        int compteurDehors;
        Timer timerDehors = new Timer();
        bool infoDehors = false;

        int compteurCancelDehors;
        Timer timerCancelDehors = new Timer();


        /// <summary>
        /// Initializes a new instance of the KinectBodyView class
        /// </summary>
        /// <param name="kinectSensor">Active instance of the KinectSensor</param>
        public KinectBodyView(KinectSensor kinectSensor)
        {
            if (kinectSensor == null)
            {
                throw new ArgumentNullException("kinectSensor");
            }

            // get the coordinate mapper
            this.coordinateMapper = kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Right Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Left Leg
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // populate body colors, one for each BodyIndex
            this.bodyColors = new List<Pen>();

            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));
            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));
            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));
            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));
            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));
            this.bodyColors.Add(new Pen(Brushes.CornflowerBlue, 6));

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.imageSource = new DrawingImage(this.drawingGroup);
        }

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Updates the body array with new information from the sensor
        /// Should be called whenever a new BodyFrameArrivedEvent occurs
        /// </summary>
        /// <param name="bodies">Array of bodies to update</param>
        public int UpdateBodyFrame(Body[] bodies)
        {
            if (bodies != null)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(Brushes.Transparent, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in bodies)
                    {
                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {
                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);
                            string actionLeft = "";
                            string actionRight = "";
                            if (body.HandLeftState == HandState.Open && body.HandRightState != HandState.Open)
                            {
                                actionLeft = "open";
                            }else if (body.HandRightState == HandState.Open && body.HandLeftState != HandState.Open)
                            {
                                actionRight = "right";
                            }
                            this.DrawHand(body.HandLeftState, jointPoints[JointType.HandLeft], dc, "left", actionLeft);
                            this.DrawHand(body.HandRightState, jointPoints[JointType.HandRight], dc, "right", actionRight);
                        }
                    }

                    // prevent drawing outside of our render area
                    this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                }
            }
            return scene;
        }

        /// <summary>
        /// Draws a body
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Draws a hand symbol if the hand is tracked: red circle = closed, green circle = opened; blue circle = lasso
        /// </summary>
        /// <param name="handState">state of the hand</param>
        /// <param name="handPosition">position of the hand</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawHand(HandState handState, Point handPosition, DrawingContext drawingContext, string main, string actionActive)
        {
            if (scene == 0 && !videoStarted)
            {

                ElapsedEventHandler timeIntro = new ElapsedEventHandler(OnTimedEventIntro);
                if (!timerIntro.Enabled)
                {
                    Debug.Print("Début intro");
                    timerIntro.Enabled = true;
                    timerIntro.Interval = 7200;
                    timerIntro.Elapsed += timeIntro;
                }
                videoStarted = true;
            }
            if (!successVideo && success)
            {
                
                if (scene == 3)
                {

                    ElapsedEventHandler timeVideo = new ElapsedEventHandler(OnTimedEventVideo);
                    if (!timerVideo.Enabled)
                    {
                        Debug.Print("Début vidéo");
                        timerVideo.Enabled = true;
                        timerVideo.Interval = 1220;
                        timerVideo.Elapsed += timeVideo;
                    }
                }
                successVideo = true;
            }
            switch (handState)
            {
                /*case HandState.Closed:
                    drawingContext.DrawEllipse(this.handClosedBrush, null, handPosition, HandSize, HandSize);
                    break;
                    */
                case HandState.Open:
                    if (!success && scene == 2)
                    {
                        drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                        if (mainActive == main)
                        {

                            positionMain = String.Format("{0}, {1}", handPosition.X, handPosition.Y);

                            if (handPosition.X > 40 && handPosition.X < 60 && handPosition.Y > 185 && handPosition.Y < 205)
                            {
                                compteurCancel = 0;
                                timerCancel.Enabled = false;
                                ElapsedEventHandler compter = new ElapsedEventHandler(OnTimedEvent);
                                if (!timer.Enabled)
                                {
                                    timer.Enabled = true;
                                    timer.Interval = 300;
                                    timer.Elapsed += compter;
                                    Debug.Print("Dans la zone" + scene);
                                }
                            }
                            else
                            {
                                ElapsedEventHandler cancel = new ElapsedEventHandler(OnTimedEventCancel);
                                if (!timerCancel.Enabled)
                                {
                                    timerCancel.Enabled = true;
                                    timerCancel.Interval = 100;
                                    timerCancel.Elapsed += cancel;
                                }
                            }
                        }
                        else
                        {
                            compteur = 0;
                            compteurCancel = 0;
                            timer.Enabled = false;
                            timerCancel.Enabled = false;
                            mainActive = main;
                        }
                    }
                    else
                    {
                        if (timer.Enabled || timerCancel.Enabled || compteur != 0 || compteurCancel != 0)
                        {
                            compteur = 0;
                            compteurCancel = 0;
                            timer.Enabled = false;
                            timerCancel.Enabled = false;
                        }
                    }
                    if (!accederMain && scene == 1)
                    {
                        drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                        if (mainActive == main)
                        {

                            positionMain = String.Format("{0}, {1}", handPosition.X, handPosition.Y);

                            if (handPosition.X > 230 && handPosition.X < 260 && handPosition.Y > 210 && handPosition.Y < 240)
                            {
                                compteurCancelFinIntro = 0;
                                timerCancelFinIntro.Enabled = false;
                                ElapsedEventHandler compterFinIntro = new ElapsedEventHandler(OnTimedEventFinIntro);
                                if (!timerFinIntro.Enabled)
                                {
                                    timerFinIntro.Enabled = true;
                                    timerFinIntro.Interval = 300;
                                    timerFinIntro.Elapsed += compterFinIntro;
                                    Debug.Print("Dans la zone");
                                }
                            }
                            else
                            {
                                ElapsedEventHandler cancelFinIntro = new ElapsedEventHandler(OnTimedEventCancelFinIntro);
                                if (!timerCancelFinIntro.Enabled)
                                {
                                    timerCancelFinIntro.Enabled = true;
                                    timerCancelFinIntro.Interval = 100;
                                    timerCancelFinIntro.Elapsed += cancelFinIntro;
                                }
                            }
                        }
                        else
                        {
                            compteurFinIntro = 0;
                            compteurCancelFinIntro = 0;
                            timerFinIntro.Enabled = false;
                            timerCancelFinIntro.Enabled = false;
                            mainActive = main;
                        }
                    }
                    else
                    {
                        if (timerFinIntro.Enabled || timerCancelFinIntro.Enabled || compteurFinIntro != 0 || compteurCancelFinIntro != 0)
                        {
                            compteurFinIntro = 0;
                            compteurCancelFinIntro = 0;
                            timerFinIntro.Enabled = false;
                            timerCancelFinIntro.Enabled = false;
                        }
                    }
                    if (!infoDehors && scene == 4)
                    {
                        drawingContext.DrawEllipse(this.handOpenBrush, null, handPosition, HandSize, HandSize);
                        if (mainActive == main)
                        {

                            positionMain = String.Format("{0}, {1}", handPosition.X, handPosition.Y);

                            if (handPosition.X > 220 && handPosition.X < 250 && handPosition.Y > 150 && handPosition.Y < 190)
                            {
                                compteurCancelDehors = 0;
                                timerCancelDehors.Enabled = false;
                                ElapsedEventHandler compterDehors = new ElapsedEventHandler(OnTimedEventDehors);
                                if (!timerDehors.Enabled)
                                {
                                    timerDehors.Enabled = true;
                                    timerDehors.Interval = 300;
                                    timerDehors.Elapsed += compterDehors;
                                    Debug.Print("Dans la zone");
                                }
                            }
                            else
                            {
                                ElapsedEventHandler cancelDehors = new ElapsedEventHandler(OnTimedEventCancelDehors);
                                if (!timerCancelDehors.Enabled)
                                {
                                    timerCancelDehors.Enabled = true;
                                    timerCancelDehors.Interval = 100;
                                    timerCancelDehors.Elapsed += cancelDehors;
                                }
                            }
                        }
                        else
                        {
                            compteurDehors = 0;
                            compteurCancelDehors = 0;
                            timerDehors.Enabled = false;
                            timerCancelDehors.Enabled = false;
                            mainActive = main;
                        }
                    }
                    else
                    {
                        if (timerDehors.Enabled || timerCancelDehors.Enabled || compteurDehors != 0 || compteurCancelDehors != 0)
                        {
                            compteurDehors = 0;
                            compteurCancelDehors = 0;
                            timerDehors.Enabled = false;
                            timerCancelDehors.Enabled = false;
                        }
                    }
                    break;

                case HandState.Lasso:
                    drawingContext.DrawEllipse(this.handLassoBrush, null, handPosition, HandSize, HandSize);

                    success = false;

                    break;
                default:
                    if (timer.Enabled && compteur > 0)
                    {
                        if (main == mainActive)
                        {

                            ElapsedEventHandler cancel = new ElapsedEventHandler(OnTimedEventCancel);
                            if (!timerCancel.Enabled)
                            {
                                timerCancel.Enabled = true;
                                timerCancel.Interval = 100;
                                timerCancel.Elapsed += cancel;
                            }
                        }
                    }
                    else
                    {
                        if (timerCancel.Enabled)
                        {
                            timerCancel.Enabled = false;
                        }
                    }
                    break;
            }
            
        }

        private void OnTimedEvent(object source, ElapsedEventArgs e)
        {
            if (timer.Enabled)
            {
                compteur++;
                Debug.Print("Acces sortie : " + compteur);
                if (compteur == 12)
                {
                    scene = 3;
                    success = true;
                    compteur = 0;
                    timer.Enabled = false;
                }
            }
        }
        private void OnTimedEventCancel(object source, ElapsedEventArgs e)
        {
            if (timerCancel.Enabled)
            {
                compteurCancel++;
                if (compteurCancel == 5)
                {
                    compteur = 0;
                    compteurCancel = 0;
                    timer.Enabled = false;
                    timerCancel.Enabled = false;
                }
            }
        }

        private void OnTimedEventVideo(object source, ElapsedEventArgs e)
        {
            Debug.Print("videoEnCours");
            if (timerVideo.Enabled)
            {
                compteurVideo++;
                if (compteurVideo == 10)
                {
                    Debug.Print("video finie");
                    compteurVideo = 0;
                    timerVideo.Enabled = false;
                    scene = 4;
                }
            }
        }
        private void OnTimedEventIntro(object source, ElapsedEventArgs e)
        {
            Debug.Print("Intro en cours");
            if (timerIntro.Enabled)
            {
                compteurIntro++;
                if (compteurIntro == 10)
                {
                    Debug.Print("Intro terminée");
                    compteurIntro = 0;
                    timerIntro.Enabled = false;
                    scene = 1;
                }
            }
        }
        private void OnTimedEventFinIntro(object source, ElapsedEventArgs e)
        {
            Debug.Print("Valider Tutoriel");
            if (timerFinIntro.Enabled)
            {
                compteurFinIntro++;
                if (compteurFinIntro == 10)
                {
                    Debug.Print("Sortie terminée");
                    compteurFinIntro = 0;
                    accederMain = true;
                    timerFinIntro.Enabled = false;
                    scene = 2;
                }
            }
        }
        private void OnTimedEventCancelFinIntro(object source, ElapsedEventArgs e)
        {
            if (timerCancelFinIntro.Enabled)
            {
                compteurCancelFinIntro++;
                if (compteurCancelFinIntro == 5)
                {
                    compteurFinIntro = 0;
                    compteurCancelFinIntro = 0;
                    timerFinIntro.Enabled = false;
                    timerCancelFinIntro.Enabled = false;
                }
            }
        }

        private void OnTimedEventDehors(object source, ElapsedEventArgs e)
        {
            Debug.Print("Voir info");
            if (timerDehors.Enabled)
            {
                compteurDehors++;
                if (compteurDehors == 10)
                {
                    Debug.Print("Fin - affichage quizz");
                    compteurDehors = 0;
                    infoDehors = true;
                    timerDehors.Enabled = false;
                    scene = 5;
                }
            }
        }
        private void OnTimedEventCancelDehors(object source, ElapsedEventArgs e)
        {
            if (timerCancelDehors.Enabled)
            {
                compteurCancelDehors++;
                if (compteurCancelDehors == 5)
                {
                    compteurDehors = 0;
                    compteurCancelDehors = 0;
                    timerDehors.Enabled = false;
                    timerCancelDehors.Enabled = false;
                }
            }
        }

        public String getPosition()
        {
            return positionMain + "";
        }


        /// <summary>
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }
    }

}
