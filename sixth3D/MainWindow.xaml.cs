using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Media.Media3D;
using System.Windows.Navigation;
using System.Windows.Shapes;
using static System.Runtime.InteropServices.JavaScript.JSType;


using Matrix4x4 = System.Numerics.Matrix4x4;
using Point = System.Windows.Point;
using Vector3 = System.Numerics.Vector3;
using System.Threading;
using System.Drawing;
using System.Reflection;
using System.Windows.Threading;
using System.Globalization;
using System.IO;
using System.ComponentModel;
using System.Xml;
using System.Windows.Media.Effects;
using Accord.Math;

namespace sixth3D
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    ///         float fov = 200f;
    public partial class MainWindow : Window
    {

        Vector3[] points = {new Vector3(0,0,0), new Vector3(100,0,0), new Vector3(100,100,0), new Vector3(0,100,0),
             new Vector3(0, 100, 100), new Vector3(0,0,100), new Vector3(100,0,100), new Vector3(100,100,100)

        };
        int[] faces = {
            0,1,2,3,
            3,0,5,4,
            0,1,6,5,
            1,2,7,6,
            3,4,7,2,
            4,5,6,7
        };
        Vector3[] points1 = {
            new Vector3(0,0,0),
            new Vector3(100,0,0),
            new Vector3(75,50,25),
            new Vector3(25,50,25),
            new Vector3(25,50,75),
            new Vector3(75,50,75),
            new Vector3(100,0,100),
            new Vector3(0,0,100),
            new Vector3(0,100,100),
            new Vector3(0,100,0),
            new Vector3(100,100,0),
            new Vector3(100,100,100),
        };
        int[] faces1 = {
            0,1,2,3,
            1,6,5,2,
            0,1,6,5,
            6,5,4,7,
            7,4,3,0,
            4,5,6,7,

            // half

            3,2,10,9,
            2,5,11,10,
            5,4,8,11,
            3,4,8,9
        };
        float fov = 100f;
        float ab = 3;
        Object3d[] WorldObjects;
        Vector3 camPos = new Vector3(-10f, 0, 0), camRot = new Vector3(0, -3f, 0);
        float speed = 5f;
        float lookSpeed = 0.1f;
        float height = 30f;

        const float fps = 240;
        float FixedInterval = 1000 / fps;

        volatile Canvas mainCanvas = new Canvas();

        DispatcherTimer ConstantTimer = new DispatcherTimer();
        DispatcherTimer RenderTimer = new DispatcherTimer();
        bool RunMainLoop = true;
        volatile Canvas debugCanvas = new Canvas();
        volatile Canvas extraCanvas = new Canvas();
        volatile Canvas FullCanvas = new Canvas();

        TextBlock tb;
        TextBlock infoBox;
        public MainWindow()
        {
            InitializeComponent();

            tb = new TextBlock();
            tb.Text = "test";

            infoBox = new TextBlock();
            tb.Visibility = Visibility.Visible;
            infoBox.Visibility = Visibility.Visible;


            Application.Current.MainWindow.Content = FullCanvas;
            var wind = Application.Current.MainWindow;

            FullCanvas.Children.Add(mainCanvas);
            FullCanvas.Children.Add(extraCanvas);
            FullCanvas.Children.Add(debugCanvas);

            FullCanvas.Children.Add(tb);
            FullCanvas.Children.Add(infoBox);



            debugCanvas.Children.Add(NewPolygon());
            debugCanvas.Children.Add(NewPolygon());


            ConstantTimer.Interval = new TimeSpan(0, 0, 0, 0, (int)FixedInterval);
            RenderTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);

            Object3d ob1 = new Object3d(points, faces);
            Object3d ob2 = new Object3d(points, faces);
            Object3d ob3 = new Object3d(points, faces, new Vector3(0.5f, 0.5f, 0.5f));
            Object3d ob5 = new Object3d(points, faces);
            Object3d ob6 = new Object3d(points, faces);





            ob1.position += new Vector3(0, 0f, 100f);
            ob2.position += new Vector3(700f, 0, 700f);
            ob3.position += new Vector3(100f, 0, 240f);
            ob5.position += new Vector3(700f, -400f, 700f);
            ob6.position += new Vector3(640f, -200, 240f);

            ob6.rotation = new Vector3(1f, 1f, 1f);

            //ob6.velocity = new Vector3(100, 100, 100);

            var col5 = new Collider(ob5.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob5.colliders = new Collider[] { col5 };

            var col6 = new Collider(ob6.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob6.colliders = new Collider[] { col6 };

            var col2 = new Collider(ob2.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob2.colliders = new Collider[] { col2 };

            Object3d ob4 = new Object3d(points1, faces1);
            ob4.position += new Vector3(0f, 0, 0f);
            ob2.rotation += new Vector3(-0.1f, 0.1f, 0.1f);
            ob5.rotation += new Vector3(0.1f, -0.1f, 0.1f);

            ob2.weight = 3000f;

            ob2.Initalize();
            ob6.Initalize();
            ob5.Initalize();

            //ob6.scale = new Vector3(0.1f, 0.1f, 0.1f);
            //ob6.angularVelocity = new Vector3(0, 1, 0);

            WorldObjects = new Object3d[] { ob3, ob2, ob1, ob4, ob5, ob6 };
            //float d = 2f / 5f;
            //float dist = 400f;
            //for (float i = 0; i < 6 * Math.PI; i += d)
            //{
            //    Object3d[] a = { new Object3d(points, faces) };
            //    a[0].position += new Vector3((float)(Math.Cos(i) * dist), 0, (float)(Math.Sin(i) * dist));
            //    Collider collider = new Collider(a[0].position,new Vector3(1,1,1), new Vector3(50,50,50),HitReaction.None);
            //    a[0].colliders = new Collider[] { collider };
            //    WorldObjects = WorldObjects.Concat(a).ToArray();

            //}




            ConstantTimer.Tick += new EventHandler(ConstantTimerUpdate);
            RenderTimer.Tick += new EventHandler(RunRender);

            ConstantTimer.Start();
            RenderTimer.Start();


            FullCanvas.Children.Add(new System.Windows.Shapes.Ellipse()
            {
                RenderTransform = new TranslateTransform() { X = wind.Width / 2f, Y = wind.Height / 2f },
                Fill = Brushes.Red,
                Height = 10f,
                Width = 10f
            }
);
        }
        public void RunRender(object sender, EventArgs e)
        {
            FullRender();
        }

        Key[] inputs = {
            Key.W,
            Key.A,
            Key.S,
            Key.D,
            Key.Up,
            Key.Right,
            Key.Left,
            Key.Down,
                Key.E,
                Key.O,
                Key.P
            };


        private void ConstantTimerUpdate(object sender, EventArgs e)
        {
            CheckKeys();

            if (holding != null)
            {
                FindForward();
                holding.velocity += ((camPos+ forward * 150f)-holding.position) * new Vector3(0.8f,1.2f,0.8f);
            }
            if (simulate)
                FullCollide();

            FindForward();
            Touch info = AdvancedRaycast(camPos, forward, 10000f);
            if (info != null)
            {
                var p = screen(camPoint(info.bodyA.position));
                infoBox.RenderTransform = new TranslateTransform()
                {
                    X = p.X,
                    Y = p.Y,
                };
                var b = info.bodyA;
                var c = b.possibleParent;
                infoBox.Text =
                    b.ID + "\n" +
                    c.velocity.ToString() + "     " + c.velocity.Length() + "\n" +
                    c.angularVelocity.ToString() + "     " + c.angularVelocity.Length() + "\n" +
                    c.weight;
            }
            else infoBox.Text = "";

        }
        Object3d holding;
        Vector3 forward = Vector3.Zero;
        int throwSpeed = 0;
        Vector3 Forward(Vector3 rotation)
        {
            return new Vector3(
                MathF.Cos(rotation.X + 90f) * MathF.Sin(rotation.Y + 90f),
                MathF.Sin(rotation.X + 90f) * MathF.Cos(rotation.Y + 90f),
                MathF.Sin(rotation.Y + 90f)
                );
        }

        void FindForward()
        {
            //forward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, Matrix4x4.CreateFromYawPitchRoll(camRot.Y, camRot.X, camRot.Z)));
            //forward = new Vector3(MathF.Cos(camRot.Y-90f),0 , MathF.Sin(camRot.Y-90f));
            Matrix4x4 first = Matrix4x4.CreateFromYawPitchRoll(-camRot.Y, 0, 0),
            second = Matrix4x4.CreateFromYawPitchRoll(0, camRot.X, 0);
            forward = Vector3.Transform(Vector3.Transform(-Vector3.UnitZ, second), first);

        }
        void CheckKeys()
        {
            if (Keyboard.IsKeyDown(inputs[3]))
            {
                camPos += new Vector3((float)Math.Cos(camRot.Y) * -speed, 0, (float)Math.Sin(camRot.Y) * -speed);
            }
            if (Keyboard.IsKeyDown(inputs[1]))
            {
                camPos += new Vector3((float)Math.Cos(camRot.Y) * speed, 0, (float)Math.Sin(camRot.Y) * speed);
            }
            var normal = new Vector3((float)Math.Cos(camRot.Y) * speed, 0, (float)Math.Sin(camRot.Y) * speed);
            if (Keyboard.IsKeyDown(inputs[0]))
            {
                camPos += new Vector3(normal.Z, 0, -normal.X);
            }
            if (Keyboard.IsKeyDown(inputs[2]))
            {
                camPos += new Vector3(-normal.Z, 0, normal.X);
            }

            camPos = new Vector3(camPos.X, height, camPos.Z);


            if (Keyboard.IsKeyDown(inputs[5]))
            {
                camRot += new Vector3(0, -lookSpeed, 0);
            }
            if (Keyboard.IsKeyDown(inputs[6]))
            {
                camRot += new Vector3(0, lookSpeed, 0);
            }

            if (Keyboard.IsKeyDown(inputs[4]))
            {
                camRot += new Vector3(lookSpeed, 0, 0);
            }
            if (Keyboard.IsKeyDown(inputs[7]))
            {
                camRot += new Vector3(-lookSpeed, 0, 0);
            }
            if (Math.Abs(camRot.Y) > 6f) camRot = new Vector3(camRot.X, 0, camRot.Z);
            //FullRender();


            if (Keyboard.IsKeyDown(inputs[8]))
            {
                if (holding == null)
                {
                    FindForward();
                    var t = AdvancedRaycast(camPos, forward, 5000f);
                    if (t != null) holding = t.bodyA.possibleParent;
                }



            }
            else
            {
                if (holding != null)
                {
                    //holding.velocity = forward;
                    holding = null;
                }

            }
            if (Keyboard.IsKeyDown(inputs[9])) simulate = false;
            if (Keyboard.IsKeyDown(inputs[10])) simulate = true;
        }
        bool simulate = true;
        Point RenderPoint(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        Matrix4x4 firstRotate;
        Matrix4x4 secondRotate;
        Matrix4x4 firstPosition;
        List<Face> faceList = new List<Face>();
        volatile int[] cullList = { };

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            FullRender(drawingContext);
        }
        Point screen(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        Vector3 camPoint(Vector3 point)
        {
            return Vector3.Transform(Vector3.Transform(Vector3.Transform(point, firstPosition), firstRotate), secondRotate);
        }
        Vector3 GetClosestPointOnLineSegment(Vector3 A, Vector3 B, Vector3 P)
        {
            Vector3 AP = P - A;       //Vector from A to P   
            Vector3 AB = B - A;       //Vector from A to B  

            float magnitudeAB = AB.LengthSquared();     //Magnitude of AB vector (it's length squared)     
            float ABAPproduct = Vector3.Dot(AP, AB);    //The DOT product of a_to_p and a_to_b     
            float distance = ABAPproduct / magnitudeAB; //The normalized "distance" from a to your closest point  

            if (distance < 0)     //Check if P projection is over vectorAB     
            {
                return A;

            }
            else if (distance > 1)
            {
                return B;
            }
            else
            {
                return A + AB * distance;
            }
        } //not my funciton
        int[] collidingLines =
        {
            0,1,
            1,2,
            2,3,
            3,0,

            0,4,
            3,7,
            2,6,
            1,5,

            4,5,
            5,6,
            6,7,
            7,4
        };

        const float airDensity = 1.204f; //kg/m^3
        const float gravity = 9.8f * 500f;
        const float defaultVeloctyDecay = 0.985f;
        const float floorYValue = 100f;
        Touch AdvancedRaycast(Vector3 linePoint, Vector3 lineDir, float distance)
        {
            Touch t = null;
            foreach (Object3d ob1 in WorldObjects) if (ob1.colliders != null) foreach (Collider mainCol in ob1.colliders)
                    {
                        for (int f = 0; f < defaultColliderFaces.Length; f += 4)
                        {
                            Vector3[] plane =
                            {
                                                    mainCol.points[defaultColliderFaces[f]],
                                                    mainCol.points[defaultColliderFaces[f+1]],
                                                    mainCol.points[defaultColliderFaces[f+2]],
                                                    mainCol.points[defaultColliderFaces[f+3]],
                                                };
                            var center = (plane[0] + plane[1] + plane[2] + plane[3]) / 4;
                            var normal = Vector3.Normalize(Vector3.Cross(plane[1] - plane[0], plane[2] - plane[0]));

                            //find where a inf line and a inf hyperplane touch
                            var point = LinePlane(center, normal, linePoint, lineDir, distance);


                            if (point != Vector3.Zero)
                            {
                                var u = Vector3.Normalize(plane[1] - plane[0]);
                                var v = Vector3.Normalize(Vector3.Cross(u, normal));

                                var x = Vector3.Dot(center - point, u); //projects the 3d point of hit to the 2d hyperplane cords
                                var y = Vector3.Dot(center - point, v);

                                if (
                                    Math.Abs(x) < mainCol.size.X
                                    &&
                                    Math.Abs(y) < mainCol.size.X
                                    )
                                {
                                    tb.Text += "\n" + x + " " + y;

                                    t = new Touch(mainCol, null, point, normal);

                                    goto FullBreak;
                                }
                            }

                        }
                    }
                FullBreak:
            {

            }
            return t;
        }

        Touch Raycast(Vector3 startingPoint, Vector3 dir, float distance)
        {
            bool touched = false;
            Touch t = null;
            foreach (Object3d ob1 in WorldObjects) if (ob1.colliders != null) foreach (Collider collider in ob1.colliders)
                    {
                        var ob2Rot = ob1.rotation;

                        Matrix4x4 m2rot = Matrix4x4.CreateFromYawPitchRoll(ob2Rot.Y, ob2Rot.X, ob2Rot.Z);
                        Matrix4x4.Invert(m2rot, out m2rot);
                        Matrix4x4 m2pos = Matrix4x4.CreateWorld(ob1.position, Vector3.UnitZ, Vector3.UnitY);

                        var p1 = Vector3.Transform(Vector3.Transform(startingPoint, m2pos), m2rot);
                        var p2 = Vector3.Transform(Vector3.Transform((dir * distance) + startingPoint, m2pos), m2rot);

                        var center = collider.position;

                        var p = GetClosestPointOnLineSegment(p1, p2, Vector3.Zero);



                        if (Math.Abs(p.X) <= collider.size.X)
                        {
                            if (Math.Abs(p.Y) <= collider.size.Y)
                            {
                                if (Math.Abs(p.Z) <= collider.size.Z)
                                {
                                    touched = true;
                                    var point = GetClosestPointOnLineSegment(startingPoint, (dir * distance) + startingPoint, center);
                                    var facePoints = new Vector3[2];
                                    for (int place = 0; place < 2; place++)
                                    {
                                        for (int pointSort = 0; pointSort < 8; pointSort++)
                                        {
                                            if (collider.points[pointSort] != facePoints[0])
                                                if (collider.points[pointSort] != facePoints[1])
                                                    if (Vector3.Distance(facePoints[place], point) > Vector3.Distance(collider.points[pointSort], point))
                                                    {
                                                        facePoints[place] = collider.points[pointSort];
                                                    }
                                        }
                                    }
                                    t = new Touch(collider, null, point, Vector3.Cross(facePoints[0], facePoints[1]));

                                    break;
                                }
                            }
                        }

                    }
            if (touched == true)
            {
                return t;
            }
            else
                return null;
        }
        int[] defaultColliderFaces =
        {
            0,1,2,3,
            0,3,7,4,
            0,4,5,1,
            5,1,2,6,
            5,4,7,6,
            7,6,2,3
        };
        Vector3 LinePlane(Vector3 planePoint, Vector3 planeNormal, Vector3 linePos, Vector3 lineDir, float distance)
        {
            lineDir = Vector3.Normalize(lineDir);

            if (Vector3.Dot(planeNormal, lineDir) == 0) return Vector3.Zero;

            if (Vector3.Distance(planePoint, linePos) > distance) return Vector3.Zero;

            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);

            if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)
                return final;
            else
                return Vector3.Zero;
        } //not my funciton
        const float floorWeight = 1000f;
        Vector3 floorPos = Vector3.UnitY * floorYValue;

        (float, float) Impulse(Collider bodyA, Vector3 point)
        {
            var normal = Vector3.UnitY;

            var a = bodyA;
            var ap = a.possibleParent;

            var pMa = ap.velocity + Vector3.Cross(ap.angularVelocity, point - a.position);
            var pMb = Vector3.Zero;
            var rel = pMa - pMb;

            var vMrel = Vector3.Dot(normal, rel);
            var e = ap.bouncyness;

            var r1 = point - a.position;
            var r2 = point - floorPos;

            var masses = (1 / ap.weight) + (1 / floorWeight);

            var rn1 = Vector3.Cross(r1, normal);
            var rn2 = Vector3.Cross(r2, normal);

            var term3 = Vector3.Dot(normal, Vector3.Cross(rn1 - a.position, r1));
            var term4 = Vector3.Dot(normal, Vector3.Cross(rn2 - floorPos, r2));

            //var Irn1 = rn1 - a.position;
            //var Irn2 = rn2 - floorPos;

            //var Irn1Ra = Vector3.Cross(Irn1, r1);
            //var Irn2Ra = Vector3.Cross(Irn2, r2);

            //var dot1 = Vector3.Dot(normal, Irn1Ra);
            //var dot2 = Vector3.Dot(normal, Irn2Ra);

            var denom = masses + term3 + term4;


            var numerator = -(1 + e) * vMrel;

            var j = numerator / denom;

            return ((float)j, vMrel);
        }
        (float, float) ImpulseB(Touch touch)
        {
            var normal = touch.normal;

            var a = touch.bodyA;
            var ap = a.possibleParent;
            var b = touch.bodyB;
            var bp = b.possibleParent;
            var e = ap.bouncyness;
            var point = touch.point;

            var pMa = ap.velocity + Vector3.Cross(ap.angularVelocity, point - a.position);
            var pMb = bp.velocity + Vector3.Cross(bp.angularVelocity, point - b.position);
            var rel = Vector3.Dot(normal, pMa - pMb);

            var numerator = -(1 + e) * rel;


            var r1 = point - a.position; //ra
            var r2 = point - floorPos; //rb

            var masses = (1 / ap.weight) + (1 / bp.weight);

            var rn1 = Vector3.Cross(r1, normal);
            var rn2 = Vector3.Cross(r2, normal);

            var term3 = Vector3.Dot(normal, Vector3.Cross(rn1 - a.position, r1));
            var term4 = Vector3.Dot(normal, Vector3.Cross(rn2 - b.position, r2));

            var denom = masses + term3 + term4;



            var j = numerator / denom;

            return ((float)j, rel);
        }
        //Get the shortest distance between a point and a plane. The output is signed so it holds information
        //as to which side of the plane normal the point is.
        public static float SignedDistancePlanePoint(Vector3 planeNormal, Vector3 planePoint, Vector3 point)
        {
            return Vector3.Dot(planeNormal, (point - planePoint));
        } //not my function
        const float RESTING_TOLERANCE = 100f;
        const float DISTANCE_BEFORE_RESTITUTION = 1f;
        const float TRUNCATE_DIGITS = 1000f;
        Vector3 TruncateVector(Vector3 vector)
        {
            return new Vector3(
                MathF.Truncate(vector.X * TRUNCATE_DIGITS) / TRUNCATE_DIGITS,
                MathF.Truncate(vector.Y * TRUNCATE_DIGITS) / TRUNCATE_DIGITS,
                MathF.Truncate(vector.Z * TRUNCATE_DIGITS) / TRUNCATE_DIGITS
                );
        }
        void FullCollide()
        {

            var watch = System.Diagnostics.Stopwatch.StartNew();
            tb.Text = "";

            var scaledTime = ConstantTimer.Interval.TotalSeconds;




            //physics below

            foreach (Object3d o in WorldObjects)
            {


                //gravity and terminal velocity below
                var scaledWeight = o.weight * o.scale.Length();
                var scaledGravity = gravity * o.gravityScale * (float)scaledTime;
                var acceleration = (o.velocity - o.oldVelocity).Length() / ConstantTimer.Interval.TotalSeconds;
                var terminalVelocity = Math.Sqrt((2 * scaledWeight * scaledGravity) / (airDensity * o.drag * acceleration));

                var gravityVector = new Vector3(0, scaledGravity, 0);
                var unscaledGravityVector = new Vector3(0, gravity, 0);

                if (false /*o.velocity.Y > terminalVelocity*/)
                {
                    tb.Text += "\n term";
                    o.velocity = new Vector3(o.velocity.X, o.velocity.Y * defaultVeloctyDecay, o.velocity.Z);
                }
                else
                    o.velocity += gravityVector;

                //floor check
                if (o.colliders != null)
                {
                    foreach (Collider c in o.colliders)
                    {
                        var v = o.position;
                        for (int i = 0; i < 8; i++) if (c.points[i].Y > v.Y) v = c.points[i];
                        if (v.Y > floorYValue)
                        {
                            var normal = -Vector3.UnitY;

                            var ar = v - o.position;
                            //o.velocity += unscaledGravityVector;
                            (float j, float rel) = Impulse(c, v);
                            //o.velocity -= unscaledGravityVector;
                            var jn = j * normal;
                            //tb.Text += "\n" + j.ToString();
                            //tb.Text += "\n" + o.angularVelocity.ToString();
                            //o.velocity = o.velocity + (jn / 1f);
                            var dist = v - new Vector3(v.X, floorYValue, v.Z);

                            o.angularVelocity = o.angularVelocity - Vector3.Cross(ar, jn*scaledGravity);
                            o.velocity += (normal * rel * (1 + o.bouncyness)) - dist;
                            //tb.Text += "\n" + o.velocity.ToString();
                            //o.position += new Vector3(0, -(v.Y - floorYValue) / 2, 0);
                        }
                    }
                    //moveing position based on velocity below
                }
            }

            //collisions below

            for (int i = 0; i < WorldObjects.Length; i++)
            {
                var current = WorldObjects[i].colliders;
                if (current != null) foreach (Collider collider in current) collider.hit = false;
            }

            for (int a = 0; a < WorldObjects.Length; a++)
            {
                var ob1 = WorldObjects[a];
                if (ob1.colliders != null)
                    foreach (Collider mainCol in ob1.colliders)
                    {
                        for (int wo = 0; wo < WorldObjects.Length; wo++)
                        {
                            if (wo != a && WorldObjects[wo].colliders != null)
                            {
                                var ob2 = WorldObjects[wo];
                                foreach (Collider collider in ob2.colliders)
                                {
                                    int currentTouch = -1;
                                    int currentTouchB = -1;
                                    var check = new Touch(mainCol, collider);

                                    //finds the touch
                                    for (int i = 0; i < mainCol.touchingColliders.Count; i++) //finds the touch 
                                    {
                                        if (check.Equals(mainCol.touchingColliders[i]))
                                        {
                                            currentTouch = i;
                                        }
                                    }
                                    for (int i = 0; i < collider.touchingColliders.Count; i++)
                                    {
                                        if (check.Equals(collider.touchingColliders[i]))
                                        {
                                            currentTouchB = i;
                                        }
                                    }

                                    bool touched = false;

                                    if (Vector3.Distance(collider.position, mainCol.position) < collider.initalDistance + mainCol.initalDistance)//if the two colliders are close
                                    {
                                        for (int i = 0; i < collidingLines.Length; i += 2)
                                        {
                                            var line0 = mainCol.points[collidingLines[i]];
                                            var line1 = mainCol.points[collidingLines[i + 1]];

                                            var lineDirrectionUnit = Vector3.Normalize(line1 - line0);
                                            var lineLength = Vector3.Distance(line0, line1);
                                            for (int f = 0; f < defaultColliderFaces.Length; f += 4)
                                            {
                                                Vector3[] plane =
                                                {
                                                    collider.points[defaultColliderFaces[f]],
                                                    collider.points[defaultColliderFaces[f+1]],
                                                    collider.points[defaultColliderFaces[f+2]],
                                                    collider.points[defaultColliderFaces[f+3]],
                                                };
                                                var center = (plane[0] + plane[1] + plane[2] + plane[3]) / 4;
                                                var normal = Vector3.Normalize(Vector3.Cross(plane[1] - plane[0], plane[2] - plane[0]));

                                                if ((center - normal - collider.position).Length() > (center - -normal - collider.position).Length()) normal = -normal;

                                                //find where a inf line and a inf hyperplane touch
                                                var point = LinePlane(center, normal, line0, lineDirrectionUnit, lineLength);
                                                var point2 = line0;

                                                if (Vector3.Distance(collider.position, line1) < Vector3.Distance(collider.position, line0))
                                                {
                                                    point2 = line1;
                                                }


                                                if (point != Vector3.Zero)
                                                {
                                                    var u = Vector3.Normalize(plane[1] - plane[0]);
                                                    var v = Vector3.Normalize(Vector3.Cross(u, normal));

                                                    var x = Vector3.Dot(center - point, u); //projects the 3d point of hit to the 2d hyperplane cords
                                                    var y = Vector3.Dot(center - point, v);

                                                    if (
                                                        Math.Abs(x) < mainCol.size.X
                                                        &&
                                                        Math.Abs(y) < mainCol.size.X
                                                        )
                                                    {
                                                        touched = true;
                                                        //tb.Text += "\n" + x + " " + y;

                                                        var t = new Touch(mainCol, collider, point, normal, point2);
                                                        var t2 = new Touch(collider, mainCol, point, normal, point2);

                                                        if (currentTouch == -1) //adds the touch class (or) changes the current one
                                                            mainCol.touchingColliders.Add(t);
                                                        else
                                                            mainCol.touchingColliders[currentTouch] = t;

                                                        if (currentTouchB == -1)
                                                            collider.touchingColliders.Add(t);
                                                        else
                                                            collider.touchingColliders[currentTouchB] = t;


                                                        goto FullBreak;
                                                    }
                                                }

                                            }
                                        }


                                    }
                                    if (touched == false) //removes the "touch" if they are not colliding
                                    {
                                        if (currentTouch != -1)
                                            mainCol.touchingColliders.RemoveAt(currentTouch);
                                        //if (currentTouchB != -1)
                                        //    collider.touchingColliders.RemoveAt(currentTouchB);

                                    }
                                }
                            FullBreak:
                                {

                                };
                            }
                        }
                        foreach (Touch touch in mainCol.touchingColliders)
                        {
                            tb.Text += "\n" + touch.bodyA.ID + "   " + touch.bodyB.ID;
                            if (touch.normal != null)
                            {
                                var point = touch.point;

                                var ob2 = touch.bodyB.possibleParent;

                                touch.normal = Vector3.Normalize(touch.normal);

                                var normal = touch.normal;
                                (float j, float rel) = ImpulseB(touch);
                                var jn = j * normal;

                                var weightRatio = ob2.weight / ob1.weight;
                                var weightRatioB = ob1.weight / ob2.weight;

                                var dist = Vector3.Dot(normal, touch.secondPoint - touch.point);

                                var ra = point - ob1.position;
                                var rb = point - ob2.position;

                                var pMa = Vector3.Dot(-normal, ob1.velocity + ob1.angularVelocity);

                                Vector3 wFinal = ob1.angularVelocity + Vector3.Cross(ob1.position - point, jn);

                                ob1.velocity += jn;
                                tb.Text += "\n " + jn.ToString();

                                //(normal * rel * (1 + o.bouncyness)) - dist;

                                var speed = Vector3.Dot(normal, ob1.velocity);

                                var pointDepth = (touch.point - touch.secondPoint).Length();

                                var reaction = Vector3.Dot(-normal, ob1.velocity) * normal;

                                var fullReaction = ((pointDepth * normal) - (normal * dist) + reaction);

                                var angularReaction = Vector3.Cross(ra, jn);


                                if (ob1.velocity.Length() < RESTING_TOLERANCE)
                                {
                                    tb.Text += "\n resting";
                                    ob1.velocity = Vector3.Zero;
                                }
                                if (dist < 0)
                                {
                                    tb.Text += "\n p";
                                    //ob1.velocity += normal * speed * (ob1.bouncyness) - (touch.secondPoint - touch.point);

                                    ob1.velocity += fullReaction * (1 + ob1.bouncyness) * weightRatio;
                                    ob1.angularVelocity += angularReaction * weightRatio;
                                    ob2.velocity -= fullReaction * (1 + ob2.bouncyness) * weightRatioB;
                                    ob2.angularVelocity -= angularReaction * weightRatioB;
                                }


                                //ob1.position = ob1.position + dot;
                                //if (ob1.velocity.Length() <= gravity && ob2.velocity.Length() <= gravity)


                                //inital velocitys: 
                            }
                        }
                    }
            }

            foreach (Object3d o in WorldObjects)
            {
                o.position += o.velocity * (float)scaledTime;
                o.velocity *= (1f - (float)scaledTime) * o.velocityDecay;

                o.rotation += o.angularVelocity * (float)scaledTime;
                if (o.rotation.Length() > (Vector3.One * (float)Math.PI * 2f).Length())
                    o.rotation -= (Vector3.One * (float)Math.PI * 2f);
                o.angularVelocity *= (1f - (float)scaledTime) * o.angularVelocityDecay;

                o.position = TruncateVector(o.position);
                o.rotation = TruncateVector(o.rotation);
                o.velocity = TruncateVector(o.velocity);
                o.angularVelocity = TruncateVector(o.angularVelocity);

                o.oldVelocity = o.velocity;
            }

            foreach (Object3d o in WorldObjects) o.FixColliders();

            watch.Stop();
            if (debugCanvas.Children.Count > 1)
            {
                var g = debugCanvas.Children[1] as Polygon;
                if (g != null)
                {
                    g.Fill = Brushes.BurlyWood;
                    g.Points = new PointCollection() { new Point(0, 10), new Point(watch.ElapsedMilliseconds * 10, 10), new Point(watch.ElapsedMilliseconds * 10, 20), new Point(0, 20) };
                    debugCanvas.Children[1] = g;
                }

            }
            else
            {
                debugCanvas.Children.Add(NewPolygon());
            }

        }
        bool renderColliders = true;

        Vector3 CosVector(Vector3 input)
        {
            return new Vector3(MathF.Cos(input.X), MathF.Cos(input.Y), MathF.Cos(input.Z));
        }
        Polygon NewPolygon() // just a simple way to make a instance of a polygon without making a ton of lines of code
        {
            Polygon p = new Polygon();
            p.Stroke = Brushes.Black;
            p.Fill = Brushes.LightBlue;
            p.StrokeThickness = 1;
            p.HorizontalAlignment = HorizontalAlignment.Left;
            p.VerticalAlignment = VerticalAlignment.Center;
            Point[] points = { new Point(0, 0), new Point(0, 100), new Point(100, 100), new Point(100, 0) };
            p.Points = new PointCollection(points);
            return p;
        }
        void UpdateFaceList()
        {
            //faceList.Clear();
            int faceListIndex = 0;
            for (int a = 0; a < WorldObjects.Length; a++)
            {
                var current = WorldObjects[a].squareFaces;
                for (int i = 0; i < current.Count; i++)
                {
                    var wantedFace = current[i].TransformAll(firstPosition).TransformAll(firstRotate).TransformAll(secondRotate);

                    if (faceListIndex >= faceList.Count)
                    {
                        faceList.Add(wantedFace);
                    }
                    else
                    {
                        faceList[faceListIndex] = wantedFace;
                    }
                    faceListIndex++;
                }
            }
            faceList.Sort(delegate (Face x, Face y)
            {
                if (x.center.Z > y.center.Z) return -1; else return 1;
            }

            );
            //faceArr = faceList.ToArray();
            if (cullList.Length < faceList.Count) Array.Resize(ref cullList, faceList.Count);
        }
        void RenderColliders()
        {
            int index = 2;
            for (int ob = 0; ob < WorldObjects.Length; ob++)
            {
                var e = WorldObjects[ob];
                if (e.colliders != null)
                {
                    for (int i = 0; i < e.colliders.Length; i++)
                    {
                        var collider = e.colliders[i];
                        if (collider.points == null) collider.GeneratePoints();
                        //var center = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(collider.position, firstPosition), firstRotate), secondRotate));
                        var tr = new Vector3[8];
                        float avg = 0;
                        var ps = new Point[8];
                        for (int a = 0; a < 8; a++)
                        {
                            tr[a] = Vector3.Transform(Vector3.Transform(Vector3.Transform(collider.points[a], firstPosition), firstRotate), secondRotate);
                            avg += tr[a].Z / 8f;
                        }
                        for (int a = 0; a < 8; a++)
                        {
                            ps[a] = RenderPoint(tr[a]);
                        }
                        if (debugCanvas.Children.Count > index)
                        {
                            var l = debugCanvas.Children[index] as Polyline;
                            l.Points = new PointCollection(ps);
                            if (avg < 0)
                                l.Stroke = Brushes.Transparent;
                            else
                            {
                                if (collider.touchingColliders.Count > 0)
                                {
                                    l.Stroke = Brushes.Red;
                                }
                                else
                                {
                                    l.Stroke = Brushes.Green;
                                }
                            }
                        }
                        else
                        {
                            System.Windows.Shapes.Polyline line = new Polyline();
                            line.Points = new PointCollection(ps);
                            line.Stroke = Brushes.Green;
                            line.StrokeThickness = 1.5;
                            if (collider.touchingColliders.Count > 0)
                            {
                                line.Stroke = Brushes.Red;
                            }
                            debugCanvas.Children.Add(line);
                        }
                        index++;

                        if (collider.touchingColliders.Count > 0)
                        {
                            foreach (Touch touching in collider.touchingColliders)
                            {
                                var r = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(touching.point, firstPosition), firstRotate), secondRotate));
                                var a = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(touching.point + touching.normal * 100f, firstPosition), firstRotate), secondRotate));


                                if (debugCanvas.Children.Count > index)
                                {
                                    var l = debugCanvas.Children[index] as Polyline;
                                    l.Points = new PointCollection() { r, a };
                                    if (avg < 0)
                                        l.Stroke = Brushes.Transparent;
                                    else
                                    {
                                        l.Stroke = Brushes.Blue;
                                    }
                                }
                                else
                                {
                                    System.Windows.Shapes.Polyline line = new Polyline();
                                    line.Points = new PointCollection() { r, a };
                                    line.Stroke = Brushes.Green;
                                    line.StrokeThickness = 1.5;
                                    debugCanvas.Children.Add(line);
                                }
                                if (touching.index != null)
                                {
                                    index++;
                                    var r2 = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(touching.bodyB.points[collidingLines[touching.index]], firstPosition), firstRotate), secondRotate));
                                    var a2 = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(touching.bodyB.points[collidingLines[touching.index + 1]], firstPosition), firstRotate), secondRotate));
                                    if (debugCanvas.Children.Count > index)
                                    {
                                        var l = debugCanvas.Children[index] as Polyline;
                                        l.Points = new PointCollection() { r2, a2 };
                                        if (avg < 0)
                                            l.Stroke = Brushes.Transparent;
                                        else
                                        {
                                            l.Stroke = Brushes.LightGoldenrodYellow;
                                        }
                                    }
                                    else
                                    {
                                        System.Windows.Shapes.Polyline line = new Polyline();
                                        line.Points = new PointCollection() { r2, a2 };
                                        line.Stroke = Brushes.Green;
                                        line.StrokeThickness = 1.5f;
                                        debugCanvas.Children.Add(line);
                                    }
                                }
                            }
                            index++;
                        }


                    }
                }
            }
            if (index < debugCanvas.Children.Count)
            {
                for (int i = index; i < debugCanvas.Children.Count; i++)
                {
                    var l = debugCanvas.Children[index] as Polyline;
                    l.Visibility = Visibility.Hidden;
                }
            }
        }
        void FullRender(DrawingContext drawingContext = null)
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();

            firstRotate = Matrix4x4.CreateFromYawPitchRoll(camRot.Y, 0, 0);
            secondRotate = Matrix4x4.CreateFromYawPitchRoll(0, camRot.X, 0);
            firstPosition = Matrix4x4.CreateWorld(camPos, Vector3.UnitZ, Vector3.UnitY);

            foreach (Object3d object3D in WorldObjects)
            {
                if (object3D != null)
                {
                    object3D.Rotate();
                }
            }
            UpdateFaceList(); //transforms all faces more info below
            /* imagine a camrea lets say at (10,10,10) xyz
            * this funciton transforms all points to make the cam at (0,0,0)
            * all other points/faces update to stay in the same place relitive to the cam
            * all of the faces points get translated by the position then the y rotaion then the x rotaion
            * after being translated or transformed (i dont know the diffrence) all points infront of the camrea will have a positive z value 
            * 
            * thats the first thing this function does
            * then the second thing is just sort the faces by the z value (higher z value first) a higher z vaule means it is farther from the cam
            */

            for (int i = 0; i < faceList.Count; i++)
            {
                var f = faceList[i];
                if (cullList[i] == 0 && f.center.Z > 0)
                {

                    //for (int vis = 0; vis < f.points.Length; vis++)
                    //{
                    //    if (f.points[vis].Z > 0) f.points[vis] = new Vector3(f.points[vis].X, f.points[vis].Y, 0);
                    //}
                    if (f.visual == null) f.ScreenPos(fov, (float)Height, (float)Width);

                    if (i > mainCanvas.Children.Count - 1)
                    {
                        Polygon p = new Polygon();
                        p.Stroke = Brushes.Black;
                        p.Fill = Brushes.LightBlue;
                        p.StrokeThickness = 1;
                        p.HorizontalAlignment = HorizontalAlignment.Left;
                        p.VerticalAlignment = VerticalAlignment.Center;
                        p.Points = new PointCollection(f.visual);
                        mainCanvas.Children.Add(p);
                    }
                    else
                    {

                        var p = mainCanvas.Children[i] as Polygon;
                        mainCanvas.Children[i].Visibility = Visibility.Visible;
                        p.Points = new PointCollection(f.visual);
                        p.Fill = f.color;
                        p.Stroke = Brushes.Black;

                    }
                }
                else
                {
                    cullList[i] = 0;
                    if (i < mainCanvas.Children.Count)
                    {
                        //var p = mainCanvas.Children[i] as Polygon;
                        mainCanvas.Children[i].Visibility = Visibility.Hidden;
                        //p.Fill = Brushes.Transparent;
                        //p.Stroke = Brushes.Transparent;
                    }
                }
            }

            if (renderColliders == true)
            {
                RenderColliders();
            }

            watch.Stop();
            if (debugCanvas.Children.Count > 0)
            {
                var g = debugCanvas.Children[0] as Polygon;
                if (g != null)
                {
                    g.Fill = Brushes.BurlyWood;
                    g.Points = new PointCollection() { new Point(0, 0), new Point(watch.ElapsedMilliseconds, 0), new Point(watch.ElapsedMilliseconds, 10), new Point(0, 10) };
                    debugCanvas.Children[0] = g;
                }
            }
            else
            {
                debugCanvas.Children.Add(NewPolygon());
            }
            RenderTimer.Interval = new TimeSpan(0, 0, 0, 0, (int)watch.ElapsedMilliseconds + 1);
            return;
        }
    }
    public class Face //faces are ONLY used to RENDER faces should not be used for raycasting or physics
    {
        public Vector3 center { get; set; }
        public Vector3[] points { get; set; }
        public Point[] visual { get; set; }
        public Point visualCenter { get; set; }
        public Brush color { get; set; }
        public Face TransformAll(Matrix4x4 m)
        {
            Vector3[] transformed = points;
            for (int i = 0; i < points.Length; i++)
            {
                transformed[i] = Vector3.Transform(points[i], m);
            }
            return new Face(transformed);
        }
        public Point[] ScreenPos(float fov, float Height, float Width)
        {
            var p = new List<Point>();
            for (int i = 0; i < points.Length; i++)
            {
                p.Add(new Point((int)((fov / (fov + points[i].Z)) * points[i].X) + (Width / 2), (int)((fov / (fov + points[i].Z)) * points[i].Y) + (Height / 2)));
            }
            FindCenter();
            visual = p.ToArray();
            Point all = new Point(0, 0);
            foreach (Point point in visual)
            {
                all = new Point(all.X + point.X, all.Y + point.Y);
            }
            all = new Point(all.X / p.Count, all.Y / p.Count);
            visualCenter = all;
            return p.ToArray();
        }

        public Vector3 FindCenter()
        {
            center = (points[0] + points[1] + points[2] + points[3]) / 4;
            return center;
        }
        public Face(Vector3[] p)
        {
            points = p;
            center = (p[0] + p[1] + p[2] + p[3]) / 4;
            color = Brushes.Orange;
        }

    }
    public class Collider //will be used for raycasts
    {
        public static int count = 0;
        public int ID { get; set; }
        public bool hit { get; set; }
        public Vector3 position { get; set; }
        public Vector3 scale { get; set; }
        public Vector3 size { get; set; }
        public bool enabled { get; set; }
        public Vector3 rotation { get; set; }
        public float initalDistance { get; set; }//if the distance between two objects is greater than this it ignores the collison between

        public Vector3[] points { get; set; }

        public Object3d possibleParent { get; set; }
        public List<Touch> touchingColliders { get; set; }
        public Vector3[] basePoints { get; set; }
        public bool resting { get; set; }




        public Vector3[] TransformAll(Matrix4x4 m)
        {
            Vector3[] transformed = points;
            for (int i = 0; i < points.Length; i++)
            {
                transformed[i] = Vector3.Transform(points[i], m);
            }
            return transformed;
        }

        public void GeneratePoints()
        {
            points = new Vector3[]
            {
                new Vector3(size.X, size.Y, size.Z) ,
                new Vector3(-size.X, size.Y, size.Z) ,
                new Vector3(-size.X, size.Y, -size.Z),
                new Vector3(size.X, size.Y, -size.Z) ,
                new Vector3(size.X, -size.Y, size.Z) ,
                new Vector3(-size.X, -size.Y, size.Z),
                new Vector3(-size.X, -size.Y, -size.Z),
                new Vector3(size.X, -size.Y, -size.Z)
            };
            basePoints = points;
            for (int i = 0; i < points.Length; i++)
            {
                points[i] *= scale;
                points[i] += position;
            }
            Matrix4x4 m = Matrix4x4.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 clone = points[i];
                points[i] = Vector3.TransformNormal(clone - position, m) * scale + position;
            }
            initalDistance = Vector3.Distance(position, points[0]);
        }
        public Collider(Vector3 position, Vector3 scale, Vector3 size)
        {
            ID = count++;
            this.position = position;
            this.scale = scale;
            this.size = size;
            initalDistance = Vector3.Distance(position, new Vector3(size.X, size.Y, size.Z) * scale);
            hit = false;
            touchingColliders = new List<Touch>();
            resting = false;


        }
    }

    public class Touch
    {
        public Collider bodyA { get; set; }
        public Collider bodyB { get; set; }
        public Vector3 point { get; set; }
        public Vector3[] pointPlane { get; set; }
        public Vector3 normal { get; set; }
        public int index { get; set; }
        public int faceIndex { get; set; }
        public Vector3 secondPoint { get; set; }
        public Touch(Collider a, Collider b, Vector3 c, Vector3 d)
        {
            bodyA = a;
            bodyB = b;
            point = c;
            normal = d;
        }
        public Touch(Collider a, Collider b, Vector3 pointOfContact, Vector3 normalVector, Vector3 point2)
        {
            bodyA = a;
            bodyB = b;
            point = pointOfContact;
            normal = normalVector;
            secondPoint = point2;
        }
        public Touch(Collider a, Collider b, Vector3 pointOfContact, Vector3 normalVector, int lineIndex, int indexOfFace)
        {
            bodyA = a;
            bodyB = b;
            point = pointOfContact;
            normal = normalVector;
            index = lineIndex;
            faceIndex = indexOfFace;
        }
        public Touch(Collider a, Collider b, Vector3 c, Vector3[] d)
        {
            bodyA = a;
            bodyB = b;
            point = c;
            pointPlane = d;
        }
        public Touch(Collider a, Collider b)
        {
            bodyA = a;
            bodyB = b;
        }
        public Touch(Touch t)
        {
            this.bodyA = t.bodyA;
            this.bodyB = t.bodyB;
            this.point = t.point;
            this.normal = t.normal;
        }
        public override bool Equals(object? a)
        {
            if (a != null)
            {
                var b = a as Touch;
                if ((bodyA.ID == b.bodyA.ID && bodyB.ID == b.bodyB.ID) || (bodyA.ID == b.bodyB.ID && bodyB.ID == b.bodyA.ID))
                {
                    return true;
                }
                else return false;
            }
            return false;
        }
    }
    //main folder for all infomation reguarding faces, collisions, rotaions and all that fun stuff
    //this stores how an object looks visualy as well as colliders and such
    public class Object3d
    {
        public static int count = 0;
        public int ID { get; set; }
        public Vector3[] basePoints { get; set; } //a list of points that should not be edited they are used for transformations and such
        public Vector3 rotation { get; set; } //effects where the points are placed in world space
        public Vector3 position { get; set; } // center and position are pretty much the same thing
        public Vector3[] worldPoints { get; set; } // where the points exist in the world
        public Vector3 center { get; set; } // center of the model follows the same rules as base points
        public Vector3 scale { get; set; } // scale of the points in the world, also effects weight
        public List<Face> squareFaces { get; set; } // list of generated faces from the var below
        public int[] orginalFaceData { get; set; }
        public Vector3 oldVelocity { get; set; }
        public Vector3 velocity { get; set; }
        public Vector3 angularVelocity { get; set; }
        public float velocityDecay { get; set; }
        public float angularVelocityDecay { get; set; }
        public float weight { get; set; }
        public float drag { get; set; }
        public float gravityScale { get; set; }
        public float bouncyness { get; set; }
        public bool allowRotaion { get; set; }
        public Collider[] colliders { get; set; }
        public void FixColliders()
        {
            if (colliders != null)
                foreach (Collider col in colliders)
                {
                    col.rotation = rotation;
                    col.scale = scale;
                    col.position = position;

                    col.possibleParent = this;
                    col.GeneratePoints();
                }
        }
        public void Rotate()
        {
            Matrix4x4 m = Matrix4x4.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            //m = Matrix4x4.CreateTranslation(center);
            //Quaternion q = Quaternion.CreateFromYawPitchRoll((float)0.1f, (float)0, (float)0);
            for (int i = 0; i < basePoints.Length; i++)
            {
                Vector3 clone = basePoints[i];
                worldPoints[i] = Vector3.TransformNormal(clone - center, m) * scale + position;
            }
            UpdateFacePosition();
        }
        public void UpdateFacePosition()
        {
            for (int i = 0; i < squareFaces.Count; i++)
            {
                Vector3[] positions = {
                worldPoints[orginalFaceData[4*i]],
                worldPoints[orginalFaceData[4*i+1]],
                worldPoints[orginalFaceData[4*i+2]],
                worldPoints[orginalFaceData[4*i+3]]
                };
                squareFaces[i].points = positions;
            }
        }
        public void Initalize()
        {
            gravityScale = 1f;
            weight = 100f;
            drag = 0.5f;
            velocityDecay = 0.99f;
            angularVelocityDecay = 0.99f;
            bouncyness = 0.2f;
            allowRotaion = true;
            oldVelocity = Vector3.Zero;
        }

        public Object3d(Vector3 pos, float mass)
        {
            position = pos;
            weight = mass;
            Initalize();
            gravityScale = 0;
        }
        public Object3d(Vector3[] basePoints1, int[] faces, Vector3 scale1)
        {
            rotation = Vector3.One;
            ID = count++;
            //Vector3[] clone = basePoints1.Concatenate(new Vector3[] { new Vector3(1f, 1f, 1f) } )
            basePoints = basePoints1;
            worldPoints = new Vector3[basePoints1.Length];
            foreach (Vector3 e in basePoints1)
            {
                center += e;
            }
            center /= basePoints1.Length;
            squareFaces = new List<Face>();
            for (int i = 0; i * 4 < faces.Length; i++)
            {
                Vector3[] positions = {
                worldPoints[faces[i]],
                worldPoints[faces[i+1]],
                worldPoints[faces[i+2]],
                worldPoints[faces[i+3]]
                };
                squareFaces.Add(new Face(positions));
            }
            orginalFaceData = faces;
            scale = scale1;
            Rotate();
        }
        public Object3d(Vector3[] basePoints1, int[] faces)
        {
            basePoints = basePoints1;
            worldPoints = new Vector3[basePoints1.Length];
            foreach (Vector3 e in basePoints1)
            {
                center += e;
            }
            center /= basePoints1.Length;
            squareFaces = new List<Face>();
            for (int i = 0; i * 4 < faces.Length; i++)
            {
                Vector3[] positions = {
                worldPoints[faces[i]],
                worldPoints[faces[i+1]],
                worldPoints[faces[i+2]],
                worldPoints[faces[i+3]]
                };
                squareFaces.Add(new Face(positions));
            }
            orginalFaceData = faces;
            scale = Vector3.One;
            Rotate();
        }

    }
}