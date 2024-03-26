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
using Quaternion = System.Numerics.Quaternion;

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
        //Vector3 camPos = new Vector3(-10f, 0, 0), camRot = new Vector3(0, -3f, 0);
        public float speed = 8f;
        public float lookSpeed = 0.05f;

        const float fps = 240;
        float FixedInterval = 1000 / fps;

        volatile Canvas mainCanvas = new Canvas();

        DispatcherTimer ConstantTimer = new DispatcherTimer();
        DispatcherTimer RenderTimer = new DispatcherTimer();
        bool RunMainLoop = true;
        volatile Canvas debugCanvas = new Canvas();
        volatile Canvas extraCanvas = new Canvas();
        volatile Canvas FullCanvas = new Canvas();
        volatile Canvas UiCanvas = new Canvas();

        ControlledCanvas controlledDebugCanvas;
        ControlledCanvas controlledUiCanvas;
        volatile MagickImage[] images = { };

        TextBlock tb;
        TextBlock infoBox;
        BitmapImage mk;
        public Camera cam;
        public Camera cam2;
        public MainWindow()
        {
            InitializeComponent();

            cam = new Camera();
            cam2 = new Camera();

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
            FullCanvas.Children.Add(UiCanvas);

            //mainCanvas.Visibility = Visibility.Hidden;

            FullCanvas.Children.Add(tb);
            FullCanvas.Children.Add(infoBox);
            //FullPathToBitmapImage(@"C:\Users\nealz\Downloads\bitmap.bmp")

            //imageCanvas.Children.Add();
            //var magic = new MagickImage(@"C:\Users\nealz\Downloads\bitmap.bmp");
            mk = FullPathToBitmapImage(@"C:\Users\nealz\Downloads\bitmap.bmp");


            controlledDebugCanvas = new ControlledCanvas(debugCanvas);
            controlledUiCanvas = new ControlledCanvas(UiCanvas);
            controlledUiCanvas.alwaysAllocated = 1000;

            var pop = new PopUp(controlledUiCanvas, PopUp.PopupType.sidebar, Width, Height, "test");

            controlledDebugCanvas.GetOrAdd("debugTimer", 0);
            controlledDebugCanvas.GetOrAdd("renderTimer", 0);
            controlledDebugCanvas.GetOrAdd("updateTimer", 0);


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


            //PathGeometry.CreateFromGeometry(new Polygon());

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
            });
            //PreviewMouseMove += new MouseEventHandler(OnPreviewMouseMove);
            //ShowCursor(false);

            //Application.Current.Run(new editor());
            edit = new Editor();
            //edit.Owner = this;
            edit.Show();
        }
        public Editor edit;
        static Image FullPathToImage(string e)
        {
            return new Image() { Source = new BitmapImage(new Uri(e)) };
        }
        static BitmapImage FullPathToBitmapImage(string e)
        {
            return new BitmapImage(new Uri(e));
        }
        [DllImport("user32.dll")]
        private static extern int ShowCursor(bool bShow);
        public void RunRender(object sender, EventArgs e)
        {
            FullRender();
        }

        public bool[] debugRenders =
        {
            false,
            false,
            false,
            false,
            true,
        };

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
                 Key.LeftShift,
                Key.LeftCtrl,

            };

        Key[] toggles =
        {
            Key.R,
            Key.Q,
            Key.G
        };
        bool[] toggleStates =
        {
            false,
            false,
            false
        };

        private void ConstantTimerUpdate(object sender, EventArgs e)
        {
            tb.Text = "";
            CheckKeys();

            if (holding != null)
            {
                //FindForward();
                
                holding.velocity += ((cam.position + cam.forward * 1500f) - holding.position);
            }
            if (simulate)
                FullCollide();

            //FindForward();
            cam.UpdateAuxiliaryVars();

            if (debugRenders[0])
            {
                var f1 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("fToFloor", 1)] as Polyline;
                f1.StrokeThickness = 1f;
                f1.Fill = Brushes.Transparent;
                f1.Stroke = Brushes.Blue;
                var floorPoint = LinePlane(floorPos, -Vector3.UnitY, cam.position, cam.forward);
                var linesize = 100f;
                f1.Points = new PointCollection(cam.VectorToView(new Vector3[] { floorPoint - Vector3.UnitZ * linesize, floorPoint + Vector3.UnitZ * linesize, floorPoint, floorPoint + Vector3.UnitX * linesize, floorPoint - Vector3.UnitX * linesize }));
            }
            if (debugRenders[1])
            {
                var plyl2 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("forward1000", 1)] as Polyline;
                plyl2.StrokeThickness = 2f;
                plyl2.Fill = Brushes.Transparent;
                plyl2.Stroke = Brushes.Red;
                plyl2.Points = new PointCollection(cam.VectorToView(new Vector3[] { Vector3.Zero, cam.position + cam.forward * 1000f }));
            }

            tb.Text += "\n" + MathF.Truncate( (float)(1000f/renderIntervalTime));

            Touch info = AdvancedRaycast(cam.position, cam.forward, 10000f);
            if (info != null)
            {
                tb.Text += "\n hit";
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
        void CheckKeys()
        {
            //
            //cam.FindForward();
            if (Keyboard.IsKeyDown(inputs[3]))
            {
                cam.position -= Vector3.Normalize(cam.right * new Vector3(1, 0, 1)) * speed;
            }
            if (Keyboard.IsKeyDown(inputs[1]))
            {
                cam.position += Vector3.Normalize(cam.right * new Vector3(1, 0, 1)) * speed;
            }

            if (Keyboard.IsKeyDown(inputs[0]))
            {
                cam.position += Vector3.Normalize(cam.forward * new Vector3(1, 0, 1)) * speed;
            }
            if (Keyboard.IsKeyDown(inputs[2]))
            {
                cam.position -= Vector3.Normalize(cam.forward * new Vector3(1, 0, 1)) * speed;
            }

            //camPos = new Vector3(camPos.X, h, camPos.Z);


            if (Keyboard.IsKeyDown(inputs[5]))
            {
                cam.rotation += new Vector3(0, lookSpeed, 0);
            }
            if (Keyboard.IsKeyDown(inputs[6]))
            {
                cam.rotation += new Vector3(0, -lookSpeed, 0);
            }

            if (Keyboard.IsKeyDown(inputs[4]))
            {
                cam.rotation += new Vector3(lookSpeed, 0, 0);
            }
            if (Keyboard.IsKeyDown(inputs[7]))
            {
                cam.rotation += new Vector3(-lookSpeed, 0, 0);
            }
            //    if (Math.Abs(camRot.Y) > 2f * MathF.PI) camRot = new Vector3(camRot.X, 2f, camRot.Z);
            //FullRender();

            if (Keyboard.IsKeyDown(inputs[8]))
            {
                if (holding == null)
                {
                    //FindForward();
                    var t = AdvancedRaycast(cam.position, cam.forward, 10000f);
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
            if (Keyboard.IsKeyDown(inputs[11])) cam.position += new Vector3(0, 10f, 0);
            if (Keyboard.IsKeyDown(inputs[12])) cam.position -= new Vector3(0, 10f, 0);

            if (Keyboard.IsKeyDown(toggles[0]))
            {
                if (toggleStates[0] == false)
                {
                    cam2.position = cam.position;
                    cam2.rotation = cam.rotation;

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
            if (Keyboard.IsKeyDown(toggles[2]))
            {
                if (toggleStates[2] == false)
                {
                    Object3d ob6 = new Object3d(points, faces);
                    ob6.Initalize();
                    ob6.position = cam.position;

                    WorldObjects = WorldObjects.Concat(new Object3d[] { ob6 }).ToArray();

                }
                toggleStates[2] = true;
            }
            else toggleStates[2] = false;
        }
        bool simulate = true;
        Point RenderPoint(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
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
            return Vector3.Transform(point, cam.WorldToEye);
        }
        float DistFromPlane(Vector3 P, Vector3 normal)
        {
            return Vector3.Dot(normal, P);
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

                        mainCol.GeneratePoints();
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
                              if (Vector3.Distance(normal + center, mainCol.position) < Vector3.Distance(center - normal, mainCol.position))
                             normal = -normal;

                            var point = LinePlane(center, normal, linePoint, lineDir);

                            var u = Vector3.Normalize(plane[1] - plane[0]);
                            var v = Vector3.Normalize(Vector3.Cross(u, normal));

                            //var projection = Vector3.Transform(point, Matrix4x4.CreateWorld(Vector3.Zero, normal, lineDir));
                           // tb.Text += "\n" + projection.ToString(); 



                            var x = Vector3.Dot(center - point, u); //projects the 3d point of hit to the 2d hyperplane cords
                            var y = Vector3.Dot(center - point, v);
                            if (debugRenders[2])
                            {
                                var plyl2 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("advRay." + f.ToString() + ".2", 1)] as Polyline;
                                plyl2.StrokeThickness = 1f;
                                plyl2.Fill = Brushes.Transparent;
                                plyl2.Stroke = Brushes.Green;
                                plyl2.Points = new PointCollection(cam.VectorToView(new Vector3[] { center, point }));
                            }
                            if (debugRenders[3])
                            {
                                var plyl3 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("advRay." + f.ToString() + ".3", 1)] as Polyline;
                                plyl3.StrokeThickness = 2f;
                                plyl3.Fill = Brushes.Transparent;
                                plyl3.Stroke = Brushes.Brown;
                                plyl3.Points = new PointCollection(cam.VectorToView(new Vector3[] { center, center + x * u, (center + x * u) + y * v }));
                            }
                            


                                if (
                                    Math.Abs(x) < 50f
                                    &&
                                    Math.Abs(y) < 50f
                                    )
                                {
                                    //  tb.Text += "\n" + x + " " + y;

                                    return new Touch(mainCol, null, point, normal);
                                }

                        }
                    }
            return t;
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
        Vector3 LinePlane(Vector3 planePoint, Vector3 planeNormal, Vector3 linePos, Vector3 lineDir)
        {
            lineDir = Vector3.Normalize(lineDir);


            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);
            // if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)

            return final;
        } //not my funciton (edited and modifyed by me)
        Vector3 LinePlane(Vector3 planePoint, Vector3 planeNormal, Vector3 linePos, Vector3 lineDir, float distance)
        {
            lineDir = Vector3.Normalize(lineDir);

            if (Vector3.Dot(planeNormal, lineDir) == 0) return Vector3.Zero;

            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);
            // if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)
            if (t > distance + 10f || (t < -10f)) return Vector3.Zero;

            return final;
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
        (Vector3 normal, Vector3 point) EvaluatePlane(Collider collider, int f)
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
            return (normal, center);
        }
        bool LinePlaneCollision(Object3d ob1, Object3d ob2, Collider mainCol, Collider collider, int currentTouch, int currentTouchB)
        {
            List<Touch> touches = new List<Touch>();
            for (int i = 0; i < collidingLines.Length; i += 2)
            {
                var line0 = mainCol.points[collidingLines[i]];
                var line1 = mainCol.points[collidingLines[i + 1]];
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


                    var point2 = line0;

                    if (Vector3.Distance(collider.position, line1) > Vector3.Distance(collider.position, line0))
                    {
                        point2 = line1;
                    }

                    var lineLength = Vector3.Distance(line0, line1);

                    var point = LinePlane(center, normal, line0, line1 - line0, lineLength);




                    if (Vector3.Distance(Vector3.Lerp(line0, line1, 0.5f), point) < lineLength / 2f)
                    {
                        var u = Vector3.Normalize(plane[1] - plane[0]);
                        var v = Vector3.Normalize(Vector3.Cross(u, normal));

                        var x = Vector3.Dot(center - point, u); //projects the 3d point of hit to the 2d hyperplane cords
                        var y = Vector3.Dot(center - point, v);

                        if (
                           MathF.Abs(x) < 50f
                            &&
                            MathF.Abs(y) < 50f
                            )
                        {
                            //tb.Text += "\n" + x + " " + y;

                            var t = new Touch(mainCol, collider, point, normal, point2);

                            touches.Add(t);
                        }
                    }

                }
            }
            if (touches.Count > 0)
            {
                Touch closest = touches[0];
                float distance = Vector3.Distance(closest.point, mainCol.position);
                for (int i = 0; i < touches.Count; i++)
                {
                    var current = touches[i];
                    var dist = Vector3.Distance(current.point, mainCol.position);

                    if (dist < distance)
                    {
                        distance = dist;
                        closest = current;
                    }
                }

                ReplaceTouch(mainCol, closest, currentTouch);
                return true;
            }

            return false;
        }
        void ReplaceTouch(Collider c1, Touch touch, int touchIndex)
        {
            if (touchIndex == -1) //adds the touch class (or) changes the current one
                c1.touchingColliders.Add(touch);
            else
                c1.touchingColliders[touchIndex] = touch;
        }

        void FullCollide()
        {
            //tb.Text = "";
            var watch = System.Diagnostics.Stopwatch.StartNew();

            var scaledTime = ConstantTimer.Interval.TotalSeconds;


            //physics below

            foreach (Object3d o in WorldObjects)
            {
                o.Rotate();
                o.FixColliders();
                
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
                        c.GeneratePoints();
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
                                        if (check.Equals(mainCol.touchingColliders[i]))
                                            currentTouch = i;
                                    for (int i = 0; i < collider.touchingColliders.Count; i++)
                                        if (check.Equals(collider.touchingColliders[i]))
                                            currentTouchB = i;

                                    bool touched = false;

                                    if (Vector3.Distance(collider.position, mainCol.position) < collider.initalDistance + mainCol.initalDistance)//if the two colliders are close
                                    {
                                        touched = LinePlaneCollision(ob1, ob2, mainCol, collider, currentTouch, currentTouchB);
                                    }
                                    if (touched == false) //removes the "touch" if they are not colliding
                                    {
                                        if (currentTouch != -1)
                                            mainCol.touchingColliders.RemoveAt(currentTouch);
                                        //if (currentTouchB != -1)
                                        //    collider.touchingColliders.RemoveAt(currentTouchB);

                                    }
                                }
                            }
                        }
                        foreach (Touch touch in mainCol.touchingColliders)
                        {
                            if (true)
                            {
                                var point = touch.point;
                                var ob = ob1;
                                var ob2 = touch.bodyB.possibleParent;

                                touch.normal = Vector3.Normalize(touch.normal);

                                var normal = touch.normal;
                                if (Vector3.Distance(ob.position + normal, ob.position) > Vector3.Distance(ob.position + -normal, ob.position))
                                    normal = -normal;
                                (float j, float rel) = ImpulseB(touch);
                                var jn = j * normal;

                                var both = ob.weight + ob2.weight;
                                var weightRatio = ob2.weight / both;
                                var weightRatioB = ob.weight / both;

                                float dist = Vector3.Dot(normal, touch.secondPoint - touch.point);

                                var ra = point - ob.position;
                                var rb = point - ob2.position;

                                var dn = -dist * normal;

                                ob.velocity += ((jn * IMPULSE_SCALE) * weightRatio) + dn;
                                ob.angularVelocity += Vector3.Cross(ra, jn) / ob.weight;
                                ob2.velocity -= ((jn * IMPULSE_SCALE) * weightRatioB) + dn;
                                ob2.angularVelocity -= Vector3.Cross(rb, jn) / ob2.weight;
                            }

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
            Polygon ui = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("debugTimer", 0)] as Polygon;
            ui.Points = new PointCollection() { new Point(0, 10), new Point(watch.ElapsedMilliseconds * 10, 10), new Point(watch.ElapsedMilliseconds * 10, 20), new Point(0, 20) };
            ui.Fill = Brushes.BurlyWood;
            //controlledDebugCanvas.SetOrAdd(ui, 1);

        }
        bool renderColliders = true;
        PointCollection V3ToScreen(Vector3[] points)
        {
            Point[] b = new Point[points.Length];
            int d = 0;
            foreach (Vector3 c in points)
            {
                var f = camPoint(c);
                b[d] = screen(f);
                d++;
            }
            return new PointCollection(b);
        }
        public double renderIntervalTime = 0.0;
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
                                avg = Math.Min(tr[a].Z,avg);
                            }
                            for (int a = 0; a < 8; a++)
                            {
                                ps[a]  = screen(tr[a] * new Vector3(-1,-1,1));
                            }
                            var l = debugCanvas.Children[controlledDebugCanvas.GetOrAdd(collider.ID.ToString() + ".line", 0)] as Polygon;
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


                            Polyline l3 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd(collider.ID.ToString() + ".vel", 1)] as Polyline;
                            l3.Points = new PointCollection( cam.VectorToView(new Vector3[] { e.position, e.velocity + e.position }));
                            l3.Fill = Brushes.Red;
                            //debugCanvas.Children[index] = l3;
                            index++;

                            //var l4 = GetDebugCanvasChild(index);

                            //l4.Points = new PointCollection(new Point[] { screen(camPoint(e.position)), screen(camPoint(e.angularVelocity*100f + e.position)) });
                            //l4.Stroke = Brushes.Blue;
                            //index++;
                            for (int tc = 0; tc < collider.touchingColliders.Count; tc++)
                            {
                                var aa = collider.touchingColliders[tc];
                                Polygon discard;
                                Polyline l1 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd(collider.ID.ToString() + "." + tc + ".dif", 1)] as Polyline;
                                l1.Points = new PointCollection(cam.VectorToView(new Vector3[] { aa.point, aa.secondPoint }));
                                l1.Stroke = Brushes.Cyan;
                                //debugCanvas.Children[index] = l1;
                                index++;

                                Polyline l2 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd(collider.ID.ToString() + "." + tc + ".norm", 1)] as Polyline;
                                l2.Points = new PointCollection(cam.VectorToView(new Vector3[] { aa.point, aa.normal * 100f + aa.point }));
                                l2.Stroke = Brushes.Yellow;
                                //debugCanvas.Children[index] = l2;
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

            Polyline l4 = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("camNorm", 1)] as Polyline;
            if (Keyboard.IsKeyDown(Key.Y))
                oldNormal = cam.forward;
                l4.Points = new PointCollection(cam.VectorToView(new Vector3[] { cam.position, cam.position + oldNormal * 100f })); 
            l4.Stroke = Brushes.Yellow;
            //debugCanvas.Children[index] = l2;
            index++;

            controlledDebugCanvas.ClearUnsed();
        }
        Vector3 oldNormal = Vector3.Zero;
        Matrix4x4 fullMatrix = new Matrix4x4();
        void FullRender(DrawingContext drawingContext = null)
        {
            var renderWatch = System.Diagnostics.Stopwatch.StartNew();
            var watch = System.Diagnostics.Stopwatch.StartNew();

            cam.width = Width;
            cam.height = Height;
            cam.output = mainCanvas;
            cam.CamOutputType = Camera.OutputType.Canvas;

            cam2.width = Width;
            cam2.height = Height;
            cam2.CamOutputType = Camera.OutputType.WorldChildList;

            
            //UpdateFaceList();
            
            if (debugRenders[4])
            {
                cam2.Render(WorldObjects, new Face[] { });
                cam.Render(WorldObjects, cam2.outputChildren.ToArray());
            }
            else
            {
                cam.Render(WorldObjects, new Face[] { });
            }

            RenderColliders();
            edit.g.AddValue((float)renderIntervalTime);
            {
                watch.Stop();
                var g = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("updateTimer", 0)] as Polygon;
                g.Fill = Brushes.BurlyWood;
                g.Points = new PointCollection() { new Point(0, 0), new Point(watch.ElapsedMilliseconds, 0), new Point(watch.ElapsedMilliseconds, 10), new Point(0, 10) };
                //debugCanvas.Children[0] = g;
                RenderTimer.Interval = new TimeSpan(0, 0, 0, 0, (int)watch.ElapsedMilliseconds + 1);
                // tb.Text += "\n" + watch.ElapsedMilliseconds;
                this.Dispatcher.BeginInvoke(
                    DispatcherPriority.Loaded,
                     new Action(() =>
                     {
                         renderWatch.Stop();
                         var a = debugCanvas.Children[controlledDebugCanvas.GetOrAdd("renderTimer", 0)] as Polygon;
                         a.Points = new PointCollection() { new Point(0, 30), new Point(renderWatch.ElapsedMilliseconds, 30), new Point(renderWatch.ElapsedMilliseconds, 40), new Point(0, 40) };// "\n Took " + renderWatch.ElapsedMilliseconds + " ms";
                         renderIntervalTime = (double)renderWatch.ElapsedMilliseconds;
                         
                     }));
            }
            return;
        }
    }
    
    public class Camera
    {
        public static int count = 0;
        public int ID { get; set; }
        public Vector3 position { get; set; } = Vector3.Zero;
        /// <summary> rad
        public Vector3 rotation { get; set; } = Vector3.UnitZ * MathF.PI;

        public Vector3 forward { get; set; } = Vector3.UnitZ;
        public Vector3 up { get; set; } = -Vector3.UnitY;
        public Vector3 right { get; set; } = Vector3.UnitX;

        public float nearPlane { get; set; } = 10f;

        public Canvas output { get; set; }
        public float fov { get; set; } = 100f;
        public Matrix4x4 WorldToEye { get; set; }
        /// <summary>
        /// normally the viewport/screen width but when used in world space it is the world frame size
        /// </summary>
        public double width { get; set; }
        /// <summary>
        /// normally the viewport/screen height but when used in world space it is the world frame size
        /// </summary>
        public double height { get; set; }
        public OutputType CamOutputType { get; set; }
        /// <summary>
        /// <
        /// </summary>
        public enum OutputType
        {
            Canvas,
            ChildList,
            CanvasAndChildList,
            WorldChildList,
            None
        };

        public Camera()
        {
            ID = count++;
        }
        public void UpdateAuxiliaryVars()
        {
            var x = Matrix4x4.CreateRotationX(rotation.X);
            var y = Matrix4x4.CreateRotationY(rotation.Y);
            var z = Matrix4x4.CreateRotationZ(rotation.Z);
            var pos = Matrix4x4.CreateTranslation(-position);
            var fullMatrix = z * x * y;

            WorldToEye = pos * z * y * x;
            forward = Vector3.Normalize(Vector3.Transform(Vector3.UnitZ, fullMatrix));
            //forward = new Vector3(forward.X, -forward.Y,forward.Z);
            right = Vector3.Normalize(Vector3.Transform(Vector3.UnitX, fullMatrix));
            up = Vector3.Normalize(Vector3.Transform(-Vector3.UnitY, fullMatrix));
        }
        public List<Face> outputChildren = new List<Face>();
        public void Render(Object3d[] input, Face[] extras)
        {
            UpdateAuxiliaryVars();

            


            List<Face> f;
            if (CamOutputType == OutputType.WorldChildList)
                f = FaceListWorld(input, extras);
            else
                f = FaceList(input, extras);

            //Matrix4x4 invWorld = new Matrix4x4();

            //if (worldProjection == true)
            //{
            //    outputChildren.Clear();
            //    Matrix4x4.Invert(WorldToEye, out invWorld);
            //}
            outputChildren.Clear();
            if (CamOutputType == OutputType.WorldChildList || CamOutputType == OutputType.ChildList || CamOutputType == OutputType.CanvasAndChildList)
            {
                
            }
            
            if (CamOutputType == OutputType.Canvas || CamOutputType == OutputType.CanvasAndChildList)
            {
                int i = 0;
                for (; i < f.Count; i++)
                {
                    var c = f[i];
                    c.visual = PointAndPlane(c.points);

                    if (i > output.Children.Count - 1)
                    {
                        output.Children.Add(
                            new Polygon()
                            {
                                Visibility = Visibility.Visible,
                                Points = new PointCollection(c.visual),
                                Stroke = Brushes.Black,
                                Fill = Brushes.LightBlue,
                                StrokeThickness = 1
                            });
                    }
                    else
                    {
                        var p = output.Children[i] as Polygon;
                        p.Points = new PointCollection(c.visual);
                        p.Visibility = Visibility.Visible;
                    }

                    if (c.z < 0 && i < output.Children.Count) output.Children[i].Visibility = Visibility.Hidden;
                }
                for (; i < output.Children.Count; i++)
                    {
                    output.Children[i].Visibility = Visibility.Hidden;
                }
            }

            if (CamOutputType == OutputType.ChildList || CamOutputType == OutputType.CanvasAndChildList)
            {
                for (int i = 0; i < f.Count; i++)
                {
                    var c = f[i];
                    c.visual = PointAndPlane(c.points);
                    outputChildren.Add(c);
                }
            }

            if (CamOutputType == OutputType.WorldChildList)
            {
                for (int i = 0; i < f.Count; i++)
                {
                    
                    var c = f[i];
                    if (c.z > 0)
                    {
                        c.points = PointAndPlaneWorld(c.points, c.z / 100f);
                        outputChildren.Add(c);
                    }
                }
            }

        }
        public List<Face> FaceListWorld(Object3d[] input, Face[] extras)// projection onto 
        {
            var output = new List<Face>();
            for (int a = 0; a < input.Length; a ++)
            {
                var faces = input[a].squareFaces;

                for (int i = 0; i < faces.Count; i ++)
                {
                    var d = new Face(faces[i]);
                    var c = d.points;

                    Vector3[] points = new Vector3[c.Length];

                    var minZ = Vector3.Dot(forward, c[0] - -position);
                    for (int b = 1; b < c.Length; b++)
                    {
                        var dist =Vector3.Dot(forward, c[b]- -position);
                        if (dist < minZ)
                            minZ = dist;

                    }
                    d.z = minZ;
                    output.Add(d);
                }
            }

                for (int i = 0; i < extras.Length; i++)
                {
                var d = new Face(extras[i]);
                    var c = d.points;

                    Vector3[] points = new Vector3[c.Length];

                    var minZ = Vector3.Dot(forward, c[0] - position);
                for (int b = 1; b < c.Length; b++)
                    {
                        var dist = Vector3.Dot(forward, c[b] - position);
                    if (dist < minZ)
                            minZ = dist;
                    }
                d.points = points;
                output.Add(d);
            }
            output.Sort(delegate (Face x, Face y)
            {
                if (x.z > y.z) return -1; else return 1;
            });
            return output;
        }
        public List<Face> FaceList(Object3d[] input, Face[] extras) //converts to eye space and sorts by z
        {
            var output = new List<Face>();
            for (int a = 0; a < input.Length; a++)
            {
                var faces = input[a].squareFaces;

                for (int i = 0; i < faces.Count; i++)
                {
                    var currentFace = faces[i].TransformAll(WorldToEye);
                    var p = currentFace.points;
                    currentFace.z = MathF.Min(MathF.Min(MathF.Min(p[0].Z, p[1].Z), p[2].Z), p[3].Z);
                    //wantedFace.z = MathF.Min(MathF.Min(MathF.Min(p[0].Z, p[1].Z), p[2].Z), p[3].Z);
                    //wantedFace.z = MathF.Max(MathF.Max(MathF.Max(p[0].Z, p[1].Z), p[2].Z), p[3].Z);
                    //(p[0].Z + p[1].Z + p[2].Z + p[3].Z) / 4;
                    output.Add(currentFace);
                }
            }
            for (int a = 0; a < extras.Length; a++)
            {
                var currentFace = extras[a].TransformAll(WorldToEye);
                var p = currentFace.points;
                currentFace.z = MathF.Min(MathF.Min(MathF.Min(p[0].Z, p[1].Z), p[2].Z), p[3].Z);
                output.Add(currentFace);
            }
            output.Sort(delegate (Face x, Face y)
            {
                if (x.z > y.z) return -1; else return 1;
            });
            return output;
        }
        Vector3 LinePlane(Vector3 linePos)
        {
            var planePoint = Vector3.UnitZ * nearPlane;
            var planeNormal = Vector3.UnitZ;
            var lineDir = Vector3.Normalize(-linePos);

            if (Vector3.Dot(planeNormal, lineDir) == 0) return Vector3.Zero;

            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);
            // if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)

            return final;
        } //not my funciton (edited and modifyed by me)
        Vector3 LinePlane(Vector3 linePos, Vector3 planeNormal, Vector3 planePoint, Vector3 lineDir)
        {
            lineDir = Vector3.Normalize(lineDir);

            if (Vector3.Dot(planeNormal, lineDir) == 0) return Vector3.Zero;

            var t = (Vector3.Dot(planeNormal, planePoint) - Vector3.Dot(planeNormal, linePos)) / Vector3.Dot(planeNormal, lineDir);
            var final = linePos + (lineDir * t);
            // if (Vector3.Distance(linePos + (lineDir * distance / 2), final) < distance / 2)

            return final;
        } //not my funciton (edited and modifyed by me)

        public Vector3[] PointAndPlaneWorld(Vector3[] face, float z)
        {

            var p = new List<Vector3>();
            var aspectRatio = (float)(height / width);

            var plane_width = MathF.Tan((MathF.PI / 180f) * fov / 2f) * nearPlane * 2f;
            var plane_height = plane_width * aspectRatio;
            var halfw = plane_width / 2f;
            var halfh = plane_height / 2f;
            var editedPosition = position + (forward * nearPlane);
            for (int i = 0; i < face.Length; i++)
            {
                var current = face[i];
                var edited = new Vector3(current.X, current.Y, current.Z);
                var point = LinePlane(edited, forward, position, edited - editedPosition);

                var x = Vector3.Dot(position - point, right) / plane_width;
                var y = Vector3.Dot(position - point, up) / plane_height;

                point += right * (float)width / 2f;
                point += up * (float)height / 2f;

                if (point.X > 1f) point.X = 1f;
                if (point.X < 0f) point.X = 0f;
                if (point.Y > 1f) point.Y = 1f;
                if (point.Y < 0f) point.Y = 0f;

                //if (x > halfw) point = point - subBackToX0 + (right * halfw);
                //if (x < -halfw) point = point - subBackToX0 + (right * -halfw);
                //if (y > halfh) point = point - subBackToY0 + (up * halfh);
                //if (y < -halfh) point = point - subBackToY0 + (up * -halfh);
                var final = position + (up * (float)height * y) + (right * (float)width * x) + (-forward * z);
                p.Add(-final);

            }
            return p.ToArray();
        }

        public Point[] PointAndPlane(Vector3[] face)
        {
            var p = new List<Point>();
            var aspectRatio = (float)(height / width);

            var plane_width = MathF.Tan((MathF.PI / 180f) * fov / 2f) * nearPlane * 2f;
            var plane_height = plane_width * aspectRatio;

            var halfw = plane_width / 2f;
            var halfh = plane_height / 2f;
            for (int i = 0; i < face.Length; i++)
            {
                var current = face[i];
                var edited = new Vector3(current.X, current.Y, current.Z);
                if (edited.Z < nearPlane) { edited.Z = nearPlane; }
                var point = LinePlane(edited);
                point = new Vector3((point.X + halfw) / plane_width, (point.Y + halfh) / plane_height, 0);
                if (point.X > 1f) point.X = 1f;
                if (point.X < 0f) point.X = 0f;
                if (point.Y > 1f) point.Y = 1f;
                if (point.Y < 0f) point.Y = 0f;

                p.Add(new Point((-point.X + 1f) * width, (-point.Y + 1f) * height));

            }
            return p.ToArray();
        }

        public Point[] VectorToView(Vector3[] p)
        {
            UpdateAuxiliaryVars();

            for (int i = 0; i < p.Length; i ++)
                p[i] = Vector3.Transform(p[i], WorldToEye);
            return PointAndPlane(p);
        }

    }
    public class PopUp
    {
        public static int count = 0;
        public int ID { get; set; }
        public ControlledCanvas targetCanvas { get; set; }
        public UIElement[] items { get; set; }
        public string[] names { get; set; }
        public enum PopupType
        {
            sidebar

        };

        public PopUp(ControlledCanvas can, PopupType tp, double width, double height, string text)
        {
            ID = count++;
            targetCanvas = can;


            float size = 1f;

            string name = "UI." + ID + ".";

            switch (tp)
            {
                case PopupType.sidebar:

                    Polygon a = targetCanvas.GetOrAddPolygon(name + "0");
                    var ab = new Point[]
                    {
                        new Point(width, height*0.75f),
                        new Point(width, height*0.8f),
                        new Point(width * 0.9f,height*0.8f),
                        new Point(width * 0.91f,height*0.75f),
                    };
                    a.Points = new PointCollection(ab);
                    a.Fill = Brushes.LightGreen;

                    TextBlock b = targetCanvas.GetOrAddTextBlock(name + "1");
                    b.RenderTransform = new TranslateTransform() { X = width * 0.9f, Y = height * 0.75f };
                    b.Text += text;
                    b.Foreground = Brushes.White;
                    b.FontWeight = FontWeights.Bold;

                    break;


            }
        }
    }
    public class ControlledCanvas
    {
        public Canvas canvas { get; }
        public List<string> names = new List<string>();
        public List<int> used = new List<int>();
        public int alwaysAllocated = 2;
        public ControlledCanvas(Canvas c)
        {
            canvas = c;
            for (int i = 0; i < canvas.Children.Count; i++)
            {
                names.Add("unknown");
                used.Add(1);
            }
        }
        /// <summary>
        /// 0 = polygon | 1 = polyline
        /// </summary>
        /// <param name="str"></param>
        /// <param name="type"></param>
        /// <returns></returns>
        public int GetOrAdd(string str, int type)
        {
            for (int i = 0; i < names.Count; i++)
            {
                if (names[i] == str)
                {
                    used[i] = 1;
                    return i;
                }
            }

            if (type == 0)
            {
                canvas.Children.Add(NewPolygon());
            }
            if (type == 1)
            {
                canvas.Children.Add(NewPolyline());
            }

            var final = canvas.Children.Count - 1;
            used.Add(1);
            names.Add(str);
            //for (; used.Count <= final;) used.Add(0);
            //for (; names.Count<= final;) names.Add("unknown");
            //names.Add(str);
            //names[final]=str;
            //used[final] = 1;
            return final;
        }
        public Polyline GetOrAddPolyline(string str)
        {
            for (int i = 0; i < names.Count; i++)
            {
                if (names[i] == str)
                {
                    used[i] = 1;
                    return canvas.Children[i] as Polyline;
                }
            }
            canvas.Children.Add(NewPolygon());
            used.Add(1);
            names.Add(str);
            return canvas.Children[canvas.Children.Count - 1] as Polyline;
        }
        public TextBlock GetOrAddTextBlock(string str)
        {
            for (int i = 0; i < names.Count; i++)
            {
                if (names[i] == str)
                {
                    used[i] = 1;
                    return canvas.Children[i] as TextBlock;
                }
            }
            canvas.Children.Add(new TextBlock());
            used.Add(1);
            names.Add(str);
            return canvas.Children[canvas.Children.Count - 1] as TextBlock;
        }
        public Polygon GetOrAddPolygon(string str)
        {
            for (int i = 0; i < names.Count; i++)
            {
                if (names[i] == str)
                {
                    used[i] = 1;
                    return canvas.Children[i] as Polygon;
                }
            }
            canvas.Children.Add(NewPolygon());
            used.Add(1);
            names.Add(str);
            return canvas.Children[canvas.Children.Count - 1] as Polygon;
        }
        public void ClearUnsed()
        {
            for (int i = canvas.Children.Count - 1; i > -1; i--)
            {
                if (used[i] == 0 && i > alwaysAllocated)
                {
                    canvas.Children.RemoveAt(i);
                    names.RemoveAt(i);
                    used.RemoveAt(i);
                    i--;
                }
            }
            for (int i = 0; i < used.Count; i++) used[i] = 0;
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
        Polyline NewPolyline()
        {
            Polyline p = new Polyline()
            {
                Stroke = Brushes.Black,
                Fill = Brushes.LightBlue,
                StrokeThickness = 1.5f,
                Points = new PointCollection()
            };
            return p;

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


        public Face SubtractAll(Vector3 position)
        {
            Vector3[] transformed = points;
            for (int i = 0; i < points.Length; i++)
            {
                transformed[i] = points[i] - position;
            }
            return new Face(transformed);
        }
        public Face TransformAll(Matrix4x4 m)
        {
            var f = new Face(this);

            Vector3[] transformed = points;
            for (int i = 0; i < points.Length; i++)
            {
                transformed[i] = Vector3.Transform(points[i], m);
            }
            f.points = transformed;
            return f;
        }
        public Face(Face f)
        {
            points = f.points;
            color = f.color;
            visual = f.visual;
            z = f.z;
            effects = f.effects;
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