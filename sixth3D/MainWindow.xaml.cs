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
using System.Drawing;
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
using System.Runtime.InteropServices;
using Accord;
using System.Text.Json.Nodes;
using Plane = System.Numerics.Plane;
using Matrix = System.Windows.Media.Matrix;
using ImageMagick;
using System.Text.RegularExpressions;

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
        volatile Canvas imageCanvas = new Canvas();
        volatile MagickImage[] images = { };

        TextBlock tb;
        TextBlock infoBox;

        Image mk;
        public MainWindow()
        {
            InitializeComponent();

            tb = new TextBlock();
            tb.Text = "test";

            //image = Image.FromFile("C:\Users\nealz\Downloads\bitmap.bmp");

            infoBox = new TextBlock();
            tb.Visibility = Visibility.Visible;
            infoBox.Visibility = Visibility.Visible;


            Application.Current.MainWindow.Content = FullCanvas;
            var wind = Application.Current.MainWindow;

            FullCanvas.Children.Add(mainCanvas);
            FullCanvas.Children.Add(extraCanvas);
            FullCanvas.Children.Add(debugCanvas);
            FullCanvas.Children.Add(imageCanvas);

            //mainCanvas.Visibility = Visibility.Hidden;

            FullCanvas.Children.Add(tb);
            FullCanvas.Children.Add(infoBox);
            //FullPathToBitmapImage(@"C:\Users\nealz\Downloads\bitmap.bmp")

            //imageCanvas.Children.Add();
            var magic = new MagickImage(@"C:\Users\nealz\Downloads\bitmap.bmp");
            mk =  FullPathToImage(@"C:\Users\nealz\Downloads\bitmap.bmp");

            debugCanvas.Children.Add(NewPolygon());
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

            //ob6.rotation = new Vector3(1f, 1f, 1f);


            //ob6.velocity = new Vector3(100, 100, 100);

            var col5 = new Collider(ob5.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob5.colliders = new Collider[] { col5 };

            var col6 = new Collider(ob6.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob6.colliders = new Collider[] { col6 };

            var col2 = new Collider(ob2.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50));
            ob2.colliders = new Collider[] { col2 };

            Object3d ob4 = new Object3d(points1, faces1);
            ob4.position += new Vector3(0f, 0, 0f);
            //ob2.rotation += new Vector3(-0.1f, 0.1f, 0.1f);
            ob5.rotation += new Vector3(0.1f, -0.1f, 0.1f);

            //ob2.weight = 3000f;

            ob2.Initalize();
            ob6.Initalize();
            ob5.Initalize();


            //ob5.bouncyness = 0;
            //ob2.bouncyness = 0;

            //ob2.scale = new Vector3(2f, 2f, 2f);

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
                RenderTransform = new TranslateTransform() { X = wind.Width / 2f + 5f, Y = wind.Height / 2f + 5f },
                Fill = Brushes.Red,
                Height = 10f,
                Width = 10f
            });
            //PreviewMouseMove += new MouseEventHandler(OnPreviewMouseMove);
            //ShowCursor(false);
        }

        static Image FullPathToImage(string e)
        {
            return new Image() {Source = new BitmapImage(new Uri(e))};
        }
        static BitmapImage FullPathToBitmapImage(string e)
        {
            return new BitmapImage(new Uri(e)) ;
        }
        [DllImport("user32.dll")]
        private static extern int ShowCursor(bool bShow);
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
                Key.P,
            };

        Key[] toggles =
        {
            Key.R,
            Key.Q
        };
        bool[] toggleStates =
        {
            false,
            false
        };




        private void ConstantTimerUpdate(object sender, EventArgs e)
        {
            CheckKeys();

            if (holding != null)
            {
                FindForward();
                holding.velocity += ((camPos + forward * 150f) - holding.position) * new Vector3(0.8f, 1.2f, 0.8f);
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

        void FindForward()
        {
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
            if (camRot.Length() > (Vector3.One * MathF.PI * 2f).Length())
                camRot -= (Vector3.One * MathF.PI * 2f);
            //    if (Math.Abs(camRot.Y) > 2f * MathF.PI) camRot = new Vector3(camRot.X, 2f, camRot.Z);
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

            if (Keyboard.IsKeyDown(toggles[0]))
            {
                if (toggleStates[0] == false)
                {
                    renderColliders = !renderColliders;

                }
                toggleStates[0] = true;
            }
            else toggleStates[0] = false;

            if (Keyboard.IsKeyDown(toggles[1]))
            {
                if (toggleStates[1] == false)
                {
                    FullCollide();

                }
                toggleStates[1] = true;
            }
            else toggleStates[1] = false;
        }
        bool simulate = true;
        Point RenderPoint(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        Matrix4x4 firstRotate;
        Matrix4x4 secondRotate;
        Matrix4x4 firstPosition;
        Matrix4x4 rotateMatrix;
        List<Face> faceList = new List<Face>();

        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            FullRender(drawingContext);
        }
        public Point screen(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        public Vector3 camPoint(Vector3 point)
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
                            var point = LinePlane(center, normal, linePoint, lineDir);


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
                                    //  tb.Text += "\n" + x + " " + y;

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
        Vector3 LinePlane2(Vector3 planeCenter, Vector3 planeNormal, Vector3 l0, Vector3 l1)
        {
            var u = l0 - l1;
            var dot = Vector3.Dot(planeNormal, u);
            if (Math.Abs(dot) > PARALLEL_PLANE_DOT_TOLERENCE)
            {
                var w = l0 - planeCenter;
                var fac = -Vector3.Dot(planeNormal, w) / dot;
                u = u * fac;

                return l0 + u;
            }
            return Vector3.Zero;
        }
        Vector3 LinePlane(Vector3 planePoint, Vector3 planeNormal, Vector3 linePos, Vector3 lineDir)
        {
            lineDir = Vector3.Normalize(lineDir);

            if (Vector3.Dot(planeNormal, lineDir) == 0) return Vector3.Zero;

            //if (Vector3.Distance(planePoint, linePos) > distance) return Vector3.Zero;

            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);

          //  if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)
                return final;
           // else
               // return Vector3.Zero;
        } //not my funciton (edited and modifyed by me)
        const float floorWeight = 1000f;
        Vector3 floorPos = Vector3.UnitY * floorYValue;

        (float, float) Impulse(Collider bodyA, Vector3 point)
        {
            var normal = Vector3.UnitY;

            var a = bodyA;
            var ap = a.possibleParent;

            var e = ap.bouncyness;

            var pMa = ap.velocity + Vector3.Cross(ap.angularVelocity, point - a.position);
            var pMb = Vector3.Zero;
            var rel = Vector3.Dot(normal, pMa - pMb);

            var ra = point - a.position; //ra
            var rb = point - floorPos; //rb

            var nTerm1 = Vector3.Dot(ap.velocity - Vector3.Zero, normal);
            var nTerm2 = Vector3.Dot(Vector3.Cross(ra, normal), ap.angularVelocity);
            var nTerm3 = Vector3.Dot(Vector3.Cross(rb, normal), ap.angularVelocity);

            var numerator = nTerm1 + nTerm2 - nTerm3;

            var masses = (1 / ap.weight) /*+ (1 / bp.weight)*/;

            var term3 = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(ra, normal), ra));
            var term4 = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(rb, normal), rb));

            var denom = masses + term3 + term4;

            var j = (numerator / denom) * -(1 + e);

            return ((float)j, rel);
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

            var ra = point - a.position; //ra
            var rb = point - b.position; //rb

            var nTerm1 = Vector3.Dot(ap.velocity - bp.velocity, normal);
            var nTerm2 = Vector3.Dot(Vector3.Cross(ra, normal), ap.angularVelocity);
            var nTerm3 = Vector3.Dot(Vector3.Cross(rb, normal), ap.angularVelocity);

            var numerator = nTerm1 + nTerm2 - nTerm3;

            var masses = (1 / ap.weight) + (1 / bp.weight);

            var term3 = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(ra, normal), ra));
            var term4 = Vector3.Dot(normal, Vector3.Cross(Vector3.Cross(rb, normal), rb));

            var denom = masses + term3 + term4;

            var j = (-(1 + e) * rel / denom);

            return ((float)j, rel);
        }
        const float RESTING_TOLERANCE = 100f;
        const float TRUNCATE_DIGITS = 1000f;
        const float IMPULSE_SCALE = 10000f;
        const float PARALLEL_DIST = 5f;
        const float PARALLEL_PLANE_DOT_TOLERENCE = 0.05f;
        Vector3 TruncateVector(Vector3 vector)
        {
            return new Vector3(
                MathF.Truncate(vector.X * TRUNCATE_DIGITS) / TRUNCATE_DIGITS,
                MathF.Truncate(vector.Y * TRUNCATE_DIGITS) / TRUNCATE_DIGITS,
                MathF.Truncate(vector.Z * TRUNCATE_DIGITS) / TRUNCATE_DIGITS
                );
        }

        Vector3 VelAtPoint(Object3d e, Vector3 point)
        {
            return e.velocity + Vector3.Cross(e.angularVelocity, (point - e.position));
        }
        static bool Separated(Vector3[] vertsA, Vector3[] vertsB, Vector3 axis)
        {
            // Handles the cross product = {0,0,0} case
            if (axis == Vector3.Zero)
                return false;

            var aMin = float.MaxValue;
            var aMax = float.MinValue;
            var bMin = float.MaxValue;
            var bMax = float.MinValue;

            // Define two intervals, a and b. Calculate their min and max values
            for (var i = 0; i < 8; i++)
            {
                var aDist = Vector3.Dot(vertsA[i], axis);
                aMin = aDist < aMin ? aDist : aMin;
                aMax = aDist > aMax ? aDist : aMax;
                var bDist = Vector3.Dot(vertsB[i], axis);
                bMin = bDist < bMin ? bDist : bMin;
                bMax = bDist > bMax ? bDist : bMax;
            }

            // One-dimensional intersection test between a and b
            var longSpan = MathF.Max(aMax, bMax) - MathF.Min(aMin, bMin);
            var sumSpan = aMax - aMin + bMax - bMin;
            return longSpan >= sumSpan; // > to treat touching as intersection
        }
        void FullCollide()
        {
            tb.Text = "";
            var watch = System.Diagnostics.Stopwatch.StartNew();

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


                            o.velocity += gravityVector; //force of gravity
                            (float j, float rel) = Impulse(c, v);
                            o.velocity -= gravityVector;

                            var dist = v - new Vector3(v.X, floorYValue, v.Z);

                            var jn = ((j * normal) /** IMPULSE_SCALE*/);



                            o.angularVelocity -= Vector3.Cross(ar, jn);
                            o.velocity += (normal * rel * (1 + o.bouncyness)) - dist;
                        }
                    }
                    //moveing position based on velocity below
                }
            }

            //collisions below
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
                                        float interSmall = 0.1f;

                                        for (int i = 0; i < collidingLines.Length; i += 2)
                                        {
                                            var line0 = mainCol.points[collidingLines[i]];
                                            var line1 = mainCol.points[collidingLines[i + 1]];

                                            var di = Vector3.Normalize(line0- ob1.position)*0.01f;
                                            var di1 = Vector3.Normalize(line1- ob1.position)*0.01f;

                                            line0 -= di;
                                            line1 -= di1;


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
                                                //var point = LinePlane(center, normal, line0-lineDirrectionUnit, lineDirrectionUnit);
                                                var point = LinePlane2(center,normal,line0,line1);
                                                if (Vector3.Distance(point, Vector3.Lerp(line1,line0,0.5f)) > lineLength/2f) break;

                                                var point2 = line0;

                                                if (Vector3.Distance(collider.position, line1) < Vector3.Distance(collider.position, line0))
                                                {
                                                    point2 = line1;
                                                }



                                                //Vector3[] collidePoints = { };

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
                            var point = touch.point;
                            var ob = ob1;
                            var ob2 = touch.bodyB.possibleParent;

                            touch.normal = Vector3.Normalize(touch.normal);

                            var normal = touch.normal;
                            if (Vector3.Distance(ob.position + normal,ob.position) > Vector3.Distance(ob.position + -normal,ob.position))
                                normal = -normal;
                            (float j, float rel) = ImpulseB(touch);
                            var jn = j * normal;

                            var both = ob.weight + ob2.weight;
                            var weightRatio = ob2.weight / both;
                            var weightRatioB = ob.weight / both;

                            float dist = Vector3.Dot(normal, touch.secondPoint - touch.point);

                            var ra = point - ob.position;
                            var rb = point - ob2.position;

                            var dn = -dist * -normal;

                            ob.velocity += ((jn * IMPULSE_SCALE) * weightRatio) + normal;// + dn;
                            ob.angularVelocity += Vector3.Cross(ra, jn) / ob.weight;
                            ob2.velocity -= ((jn * IMPULSE_SCALE) * weightRatioB) + normal;// + dn;
                            ob2.angularVelocity -= Vector3.Cross(rb, jn)/ob2.weight;


                            //if (ob1.velocity.Length() > 1f)
                            //{

                            //    Vector3 vectorLine = Vector3.Normalize(Vector3.Cross(Vector3.Cross(normal, Vector3.UnitY), normal));

                            //    var staticFriction = touch.bodyA.staticFriction;
                            //    var keneticFriction = touch.bodyA.keneticFriction;

                            //    var dirrectionalVelocity = Vector3.Dot(ob1.velocity, vectorLine);

                            //    ob1.velocity -= dirrectionalVelocity * vectorLine * keneticFriction;
                            //}


                        }
                    }
            }

            foreach (Object3d o in WorldObjects)
            {
                o.position = TruncateVector(o.position);
                o.rotation = TruncateVector(o.rotation);
                o.velocity = TruncateVector(o.velocity);
                o.angularVelocity = TruncateVector(o.angularVelocity);

                o.position += o.velocity * (float)scaledTime;
                o.velocity *= (1f - (float)scaledTime) * o.velocityDecay;

                o.rotation += o.angularVelocity * (float)scaledTime;
                if (o.rotation.Length() > (Vector3.One * (float)Math.PI * 2f).Length())
                    o.rotation -= (Vector3.One * (float)Math.PI * 2f);
                o.angularVelocity *= (1f - (float)scaledTime) * o.angularVelocityDecay;

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
        Polyline GetDebugCanvasChild(int i)
        {
            if (debugCanvas.Children.Count > i)
            {
                var l = debugCanvas.Children[i] as Polyline;
                return l;
            }
            else
            {
                var a = new Polyline()
                {
                    StrokeThickness = 1.5,
                    Stroke = Brushes.DeepPink,
                    Visibility = Visibility.Visible,
                    Points = new PointCollection(4)
                };
                debugCanvas.Children.Add(a);
                return a;
            }
        }
        void FullDrawDebugLine(Vector3[] points, Brush color, int index)
        {
            var a = GetDebugCanvasChild(index);
            Point[] b = new Point[points.Length];
            int d = 0;
            float e = 0f;
            foreach (Vector3 c in points)
            {
                var f = camPoint(c);
                b[d] = screen(f);
                e += f.Z;
                d++;
            }
            e /= points.Length;
            if (e < 0f) a.Visibility = Visibility.Hidden; else a.Visibility = Visibility.Visible;
            a.Points = new PointCollection(b);
            a.Stroke = color;
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
                    var p = wantedFace.points;

                    //current[i].z = 
                    wantedFace.z = MathF.Min(MathF.Min(MathF.Min(p[0].Z, p[1].Z), p[2].Z), p[3].Z);
                    wantedFace.z = (p[0].Z + p[1].Z + p[2].Z + p[3].Z) / 4;

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
                if (x.z > y.z) return -1; else return 1;
            }

            );
            //faceArr = faceList.ToArray();
            //if (cullList.Length < faceList.Count) Array.Resize(ref cullList, faceList.Count);
        }
        void RenderColliders()
        {
            int index = 3;
            if (debugCanvas.Children.Count >= 3)
                for (int ob = 0; ob < WorldObjects.Length; ob++)
                {
                    var e = WorldObjects[ob];
                    if (e.colliders != null)
                    {
                        for (int i = 0; i < e.colliders.Length; i++)
                        {
                            var collider = e.colliders[i];
                            if (collider.points == null) collider.GeneratePoints();
                            var tr = new Vector3[8];
                            float avg = 0;
                            var ps = new Point[8];
                            for (int a = 0; a < 8; a++)
                            {
                                tr[a] = camPoint(collider.points[a]);
                                avg += tr[a].Z / 8f;
                            }
                            for (int a = 0; a < 8; a++)
                            {
                                ps[a] = screen(tr[a]);
                            }
                            var l = GetDebugCanvasChild(index);
                            l.Points = new PointCollection(ps);
                            if (renderColliders == false)
                            {
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
                            else l.Stroke = Brushes.Transparent;
                            index++;
                            if (renderColliders == true)
                            {
                                if (debugCanvas.Children.Count > index)
                                    debugCanvas.Children[index].Visibility = Visibility.Hidden;
                            }
                            else
                            {
                                var l3 = new Vector3[] { e.position, e.velocity + e.position };
                                FullDrawDebugLine(l3, Brushes.Red, index);
                            }
                            index++;

                            //var l4 = GetDebugCanvasChild(index);

                            //l4.Points = new PointCollection(new Point[] { screen(camPoint(e.position)), screen(camPoint(e.angularVelocity*100f + e.position)) });
                            //l4.Stroke = Brushes.Blue;
                            //index++;

                            if (collider.touchingColliders.Count > 0)
                            {
                                var aa = collider.touchingColliders[0];


                                if (renderColliders == true)
                                {
                                    if (debugCanvas.Children.Count > index)
                                        debugCanvas.Children[index].Visibility = Visibility.Hidden;
                                }
                                else
                                {
                                    var l1 = new Vector3[] { aa.point, aa.secondPoint };
                                    FullDrawDebugLine(l1, Brushes.Cyan, index);
                                }
                                index++;

                                if (renderColliders == true)
                                {
                                    if (debugCanvas.Children.Count > index)
                                        debugCanvas.Children[index].Visibility = Visibility.Hidden;
                                }
                                else
                                {
                                    var l2 = new Vector3[] { aa.point, aa.normal * 100f + aa.point };
                                    FullDrawDebugLine(l2, Brushes.Yellow, index);
                                }
                                index++;
                            }

                        }
                    }
                }
            //for (int i = index; i < debugCanvas.Children.Count; i++)
            //{
            //    var l = debugCanvas.Children[i] as Polyline;
            //    l.Visibility = Visibility.Hidden;
            //}
        }
        private void CursorPos(int x, int y)
        {
            SetCursorPos(x, y);
        }

        [DllImport("User32.dll")]
        private static extern bool SetCursorPos(int x, int y);

        float sens = 0.005f;

        //private void OnPreviewMouseMove(object sender, MouseEventArgs e)
        //{
        //    if (!Keyboard.IsKeyDown(inputs[11])) { 
        //        var wind = Application.Current.MainWindow;
        //        var middle = new Vector2((float)wind.Width/2f,(float)wind.Height/2f);
        //        var pos = Mouse.GetPosition(mainCanvas);

        //        camRot += new Vector3((float)-( middle.Y -pos.Y),(float)( middle.X-pos.X),0) * sens;
        //        var realPos = wind.PointToScreen(new Point(middle.X,middle.Y));
        //        //SetCursorPos((int)(realPos.X), (int)(realPos.Y));
        //        //Mouse
        //        //SetCursorPos((int)(realPos.X), (int)(realPos.Y));
        //    }
        //}

        //System.Windows.Media.Effects.Effect

        //Image Skew(Image im, Point p0, Point p1, Point p2, Point p3, Point offset)
        //{
        //    var m11 = p2.X - p0.X;
        //    var m12 = p2.Y - p0.Y;
        //    var m21 = p1.X - p0.X;
        //    var m22 = p1.Y - p0.Y;

        //    var denom = (m11 * m22 - m12 * m21);

        //    var a = (m22 * p3.X - m21 * p3.Y + m21 * offset.Y - m22 * offset.X) / denom;
        //    var b = (m11 * p3.Y - m12 * p3.X + m12 * offset.X - m11 * offset.Y) / denom;


        //}
        //Image Skew(Image im, Point p0, Point p1, Point p2, Point p3)
        //{
        //    new DrawingContext();

        //}

        Matrix M4x4ToM3D(Matrix4x4 m)
        {
            var mat = new Matrix(
                m.M11,m.M12,//m.M13,m.M14,
                m.M21,m.M22,0,0//,m.M23,m.M24,
                //m.M31,m.M32,m.M33,m.M34,
                //m.M41,m.M42,m.M43,m.M44
                );
            //var mat = new Matrix(
    //m.M11, m.M12, m.M13, m.M14,
    //m.M21, m.M22, m.M23, m.M24,
    //m.M31, m.M32, m.M33, m.M34,
    //m.M41, m.M42, m.M43, m.M44
    //);

            return mat;
        }
        void FullRender(DrawingContext drawingContext = null)
        {
            var renderWatch = System.Diagnostics.Stopwatch.StartNew();
            var watch = System.Diagnostics.Stopwatch.StartNew();

            firstRotate = Matrix4x4.CreateFromYawPitchRoll(camRot.Y, 0, 0);
            secondRotate = Matrix4x4.CreateFromYawPitchRoll(0, camRot.X, 0);
            firstPosition = Matrix4x4.CreateWorld(camPos, Vector3.UnitZ, Vector3.UnitY);
            //Matrix4x4.Invert(firstPosition, out firstPosition);
            //rotateMatrix = Matrix4x4.Add(firstRotate,secondRotate);


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
            ImageBrush a = new ImageBrush(mk.Source)
            {

                
            };
            
            for (int i = 0; i < faceList.Count; i++)
            {

                var f = faceList[i];

                if (f.z > 0)
                {
                    //tb.Text += "\n added";
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
                        var pc = new PointCollection(f.visual);
                        p.Points = pc;
                        p.Visibility = Visibility.Visible;
                        if (f.fullUpdate == true)
                        {
                            f.fullUpdate = false;
                           // p.Fill = a;
                            //ttttp.Stroke = Brushes.Black;
                            if (f.effects != null)
                            {
                                p.Effect = f.effects;
                            }
                            if (f.image != null)
                            {
                                if (f.imageIndex == null)
                                {
                                    imageCanvas.Children.Add(new Image());
                                    f.imageIndex = imageCanvas.Children.Count - 1;

                                } else
                                {
                                    
                                }
                            }
                        }




                    }
                }
                else
                {
                    if (i < mainCanvas.Children.Count)
                    {
                        //var p = mainCanvas.Children[i] as Polygon;
                        mainCanvas.Children[i].Visibility = Visibility.Hidden;
                        //p.Fill = Brushes.Transparent;
                        //p.Stroke = Brushes.Transparent;
                    }
                }
            }

            RenderColliders();

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
            // tb.Text += "\n" + watch.ElapsedMilliseconds;
            this.Dispatcher.BeginInvoke(
                DispatcherPriority.Loaded,
                 new Action(() =>
                 {
                     renderWatch.Stop();
                     var a = debugCanvas.Children[2] as Polygon;
                     a.Points = new PointCollection() { new Point(0, 30), new Point(renderWatch.ElapsedMilliseconds, 30), new Point(renderWatch.ElapsedMilliseconds, 40), new Point(0, 40) };// "\n Took " + renderWatch.ElapsedMilliseconds + " ms";
                 }));

            return;
        }
    }
    public class Face //faces are ONLY used to RENDER faces should not be used for raycasting or physics
    {
        
        public Vector3[] points { get; set; }
        public Point[] visual { get; set; }
        public float z { get; set; }
        public Brush color { get; set; }
        public Effect effects { get; set; }
        public bool fullUpdate { get; set; }

        public Image image { get; set; } 
        public int imageIndex { get; set; }

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
            visual = p.ToArray();
            return p.ToArray();
        }
        public Face(Vector3[] p)
        {
            points = p;
            color = Brushes.Orange;
            fullUpdate = true;
            //image = im.Clone();
        }
    }
    public class Collider //will be used for raycasts
    {
        public static int count = 0;
        public int ID { get; set; }
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
        public float staticFriction { get; set; } = 0.5f;
        public float keneticFriction { get; set; } = 0.3f;
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
            touchingColliders = new List<Touch>();


        }
    }

    public class Touch
    {
        public Collider bodyA { get; set; }
        public Collider bodyB { get; set; }
        public Vector3 point { get; set; }
        public Vector3 normal { get; set; }
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
        public Vector3 rotation { get; set; } = Vector3.Zero; //effects where the points are placed in world space
        public Vector3 position { get; set; } = Vector3.Zero; // center and position are pretty much the same thing
        public Vector3[] worldPoints { get; set; } // where the points exist in the world
        public Vector3 center { get; set; } // center of the model follows the same rules as base points
        public Vector3 scale { get; set; } // scale of the points in the world, also effects weight
        public List<Face> squareFaces { get; set; } // list of generated faces from the var below
        public int[] orginalFaceData { get; set; }
        public Vector3 oldVelocity { get; set; } = Vector3.Zero;
        public Vector3 velocity { get; set; }
        public Vector3 angularVelocity { get; set; }
        public float velocityDecay { get; set; }
        public float angularVelocityDecay { get; set; }
        public float weight { get; set; }
        public float drag { get; set; }
        public float gravityScale { get; set; }
        public float bouncyness { get; set; }
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
        //public void ForceAtPosition(Vector3 pos, Vector3 force)
        //{
        //    velocity += force;
        //    angularVelocity += Vector3.Cross(pos - ( pos+force),);
        //}
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
            velocityDecay = 0.985f;
            angularVelocityDecay = 0.99f;
            bouncyness = 0.2f;
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