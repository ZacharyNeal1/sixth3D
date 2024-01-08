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

        const float fps = 60;
        float FixedInterval = 1000/fps;

        volatile Canvas mainCanvas = new Canvas();

        DispatcherTimer ConstantTimer = new DispatcherTimer();
        DispatcherTimer RenderTimer = new DispatcherTimer();
        bool RunMainLoop = true;
        volatile Canvas debugCanvas = new Canvas();
        volatile Canvas extraCanvas = new Canvas();
        volatile Canvas FullCanvas = new Canvas();
        public MainWindow()
        {
            InitializeComponent();
            Application.Current.MainWindow.Content = FullCanvas;

            FullCanvas.Children.Add(mainCanvas);
            
            FullCanvas.Children.Add(extraCanvas);
            FullCanvas.Children.Add(debugCanvas);

            debugCanvas.Children.Add(NewPolygon());
            debugCanvas.Children.Add(NewPolygon());


            ConstantTimer.Interval = new TimeSpan(0, 0, 0, 0, (int)FixedInterval);
            RenderTimer.Interval = new TimeSpan(0, 0, 0, 0, 100);

            Object3d ob1 = new Object3d(points, faces);
            Object3d ob2 = new Object3d(points, faces);
            Object3d ob3 = new Object3d(points, faces,new Vector3(0.5f,0.5f,0.5f));
            Object3d ob5 = new Object3d(points, faces);
            Object3d ob6 = new Object3d(points, faces);





            ob1.position += new Vector3(0, 100f, 100f);
            ob2.position += new Vector3(690f, 0, 150f);
            ob3.position += new Vector3(100f, 0, 240f);
            ob5.position += new Vector3(600f, 0, 400f);
            ob6.position += new Vector3(640f, -50, 240f);

            ob6.rotation = new Vector3(1f,1f,1f);

            var col5 = new Collider(ob5.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50), HitReaction.None, MovementType.All);
            ob5.colliders = new Collider[] { col5 };

            var col6 = new Collider(ob6.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50), HitReaction.None, MovementType.All);
            ob6.colliders = new Collider[] { col6 };

            var col2 = new Collider(ob2.position, new Vector3(1, 1, 1), new Vector3(50, 50, 50), HitReaction.None, MovementType.All);
            ob2.colliders = new Collider[] { col2 };

            Object3d ob4 = new Object3d(points1, faces1);
            ob4.position += new Vector3(0f, 0, 0f);

            WorldObjects = new Object3d[] { ob3, ob2, ob1, ob4, ob5, ob6 };
            float d = 2f / 20f;
            float dist = 400f;
            for (float i = 0; i < 6f; i += d)
            {
                Object3d[] a = { new Object3d(points, faces) };
                a[0].position += new Vector3((float)(Math.Cos(i) * dist), 0, (float)(Math.Sin(i) * dist));
                WorldObjects = WorldObjects.Concat(a).ToArray();

            }
            



            ConstantTimer.Tick += new EventHandler(ConstantTimerUpdate);
            RenderTimer.Tick += new EventHandler(RunRender);

            ConstantTimer.Start();
            RenderTimer.Start();
        }
        public void RunRender(object sender, EventArgs e)
        {
            FullRender();
            //InvalidateVisual();
                //Thread.Sleep(100);
        }

        Key[] inputs = {
            Key.W,
            Key.A,
            Key.S,
            Key.D,
            Key.Up,
            Key.Right,
            Key.Left,
            Key.Down
            };

        private void ConstantTimerUpdate(object sender, EventArgs e)
        {
            CheckKeys();
            FullCollide();
        }
        //protected override void OnKeyUp(KeyEventArgs e)
        //{
        //    base.OnKeyUp(e);
        //    CheckKeys();
        //}
        //protected override void OnKeyDown(KeyEventArgs e)
        //{
        //    base.OnKeyDown(e);
        //    CheckKeys();
        //}
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
        }
        Point RenderPoint(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        Matrix4x4 firstRotate;
        Matrix4x4 secondRotate;
        Matrix4x4 firstPosition;
        List<Face> faceList = new List<Face>();
        volatile int[] cullList = { };
        
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
        protected override void OnRender(DrawingContext drawingContext)
        {
            base.OnRender(drawingContext);
            FullRender(drawingContext);
        } 
        public static Vector2 LineInterection(Point p1, Point p2, Point p3, Point p4)
        {
            var eq1 =( p1.X * p2.Y - p1.Y * p2.X );
            var eq2 = (p3.X - p4.X);
            var eq3 = (p1.X - p2.X);
            var eq4 = (p3.X * p4.Y - p3.Y * p4.X);

            var denom = (p1.X - p2.X) * (p3.Y - p4.Y) - (p1.Y - p2.Y) * (p3.X - p4.X);

            var eq5 = (p3.Y - p4.Y);
            var eq6 = (p1.Y - p2.Y);
            return new Vector2(
                (float)( (eq1 * eq2 - eq3 * eq4) / denom),
                (float)( (eq1 * eq5 - eq6 * eq4) / denom)
                );
            
        }
        Point screen(Vector3 a)
        {
            return new Point((int)((fov / (fov + a.Z)) * a.X) + (Width / 2), (int)((fov / (fov + a.Z)) * a.Y) + (Height / 2));
        }
        void FullCollide()
        {
            var watch = System.Diagnostics.Stopwatch.StartNew();

            foreach (Object3d o in WorldObjects) o.FixColliders();

            for (int i = 0; i < WorldObjects.Length; i ++)
            {
                var current = WorldObjects[i].colliders;
                if (current != null) foreach (Collider collider in current) collider.hit = false;
            }


            for (int a = 0; a < WorldObjects.Length; a++)
            {
                var ob1 = WorldObjects[a];
                if (ob1.colliders != null)
                {
                    foreach (Collider mainCol in ob1.colliders) {

                        for (int i = 0; i < WorldObjects.Length; i++)
                        {
                            if (i != a && WorldObjects[i].colliders != null)
                            {
                                var ob2 = WorldObjects[i];
                                foreach (Collider collider in ob2.colliders)
                                {
                                    if (Vector3.Distance(collider.position, mainCol.position) < collider.initalDistance + mainCol.initalDistance)
                                    {
                                        var colPoints1 = mainCol.points;
                                        var colPoints2 = collider.points;

                                        var ob1Rot = ob1.rotation;
                                        var ob2Rot = ob2.rotation;
                                        Matrix4x4 m1rot = Matrix4x4.CreateFromYawPitchRoll(ob1Rot.Y, ob1Rot.X,ob1Rot.Z);
                                        Matrix4x4 m2rot = Matrix4x4.CreateFromYawPitchRoll(ob2Rot.Y, ob2Rot.X, ob2Rot.Z);
                                        Matrix4x4.Invert(m1rot,out m1rot);
                                        Matrix4x4 m1pos = Matrix4x4.CreateWorld(ob1.position, Vector3.UnitZ, Vector3.UnitY);
                                        Matrix4x4 m2pos = Matrix4x4.CreateWorld(ob2.position, Vector3.UnitZ, Vector3.UnitY);

                                        

                                        for (int b = 0; b < colPoints2.Length; b++)
                                        {
                                            bool[] hits = { false, false, false };
                                            var transformed = Vector3.Transform(Vector3.Transform(colPoints2[b], m1rot), m1pos);
                                            if (Math.Abs(transformed.X) < mainCol.size.X)
                                            {
                                                hits[0] = true;
                                            }
                                            if (Math.Abs(transformed.Y) < mainCol.size.Y)
                                            {
                                                hits[1] = true;
                                            }
                                            if (Math.Abs(transformed.Z) < mainCol.size.Z)
                                            {
                                                hits[2] = true;
                                            }
                                            if (hits[0] && hits[1] && hits[2])
                                            {
                                                if (!mainCol.touchingColliders.Contains(collider))
                                                {
                                                    mainCol.touchingColliders.Add(collider);
                                                }
                                                if (!collider.touchingColliders.Contains(mainCol))
                                                {
                                                    collider.touchingColliders.Add(mainCol);
                                                }
                                                break;
                                            }

                                        }



                                        //double[] ob2Values = new double[6] //sets all the values to the avrage i guess
                                        //{ob2.position.X, ob2.position.X,
                                        // ob2.position.Y, ob2.position.Y,
                                        // ob2.position.Z, ob2.position.Z};
                                        //double[] ob1Values = new double[6]
                                        //{ob1.position.X, ob1.position.X,
                                        // ob1.position.Y, ob1.position.Y,
                                        // ob1.position.Z, ob1.position.Z};

                                        //for (int finder = 0; finder < 8; finder++)
                                        //{
                                        //    ob1Values[0] = Math.MinMagnitude(ob1Values[0], colPoints1[finder].X);
                                        //    ob1Values[2] = Math.MinMagnitude(ob1Values[2], colPoints1[finder].Y);
                                        //    ob1Values[4] = Math.MinMagnitude(ob1Values[4], colPoints1[finder].Z);
                                        //    ob1Values[1] = Math.MaxMagnitude(ob1Values[1], colPoints1[finder].X);
                                        //    ob1Values[3] = Math.MaxMagnitude(ob1Values[3], colPoints1[finder].Y);
                                        //    ob1Values[5] = Math.MaxMagnitude(ob1Values[5], colPoints1[finder].Z);

                                        //    ob2Values[0] = Math.MinMagnitude(ob2Values[0], colPoints2[finder].X);
                                        //    ob2Values[2] = Math.MinMagnitude(ob2Values[2], colPoints2[finder].Y);
                                        //    ob2Values[4] = Math.MinMagnitude(ob2Values[4], colPoints2[finder].Z);
                                        //    ob2Values[1] = Math.MaxMagnitude(ob2Values[1], colPoints2[finder].X);
                                        //    ob2Values[3] = Math.MaxMagnitude(ob2Values[3], colPoints2[finder].Y);
                                        //    ob2Values[5] = Math.MaxMagnitude(ob2Values[5], colPoints2[finder].Z);
                                        //}

                                    }
                                }
                            }
                        }
                    }
                }
            }




            watch.Stop();
            if (debugCanvas.Children.Count > 1)
            {
                var g = debugCanvas.Children[1] as Polygon;
                if (g != null)
                {
                    g.Fill = Brushes.BurlyWood;
                    g.Points = new PointCollection() { new Point(0, 10), new Point(watch.ElapsedMilliseconds, 10), new Point(watch.ElapsedMilliseconds, 20), new Point(0, 20) };
                    debugCanvas.Children[1] = g;
                }

            }
            else
            {
                debugCanvas.Children.Add(NewPolygon());
            }

        }
        bool renderColliders = true;

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

                    if (i > mainCanvas.Children.Count-1)
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
                        p.Fill =  f.color;
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
                

                int index = 2;
                for (int ob = 0; ob < WorldObjects.Length; ob++)
                {
                    var e = WorldObjects[ob];
                    if (e.colliders != null) {
                        for (int i = 0; i < e.colliders.Length; i++)
                        {
                            var collider = e.colliders[i];
                            if (collider.points == null) collider.GeneratePoints();
                            //var center = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(collider.position, firstPosition), firstRotate), secondRotate));
                            var tr = new Vector3[8];
                            float avg =0;
                            var ps = new Point[8];
                            for (int a = 0; a < 8; a++)
                            {
                                tr[a] = Vector3.Transform(Vector3.Transform(Vector3.Transform(collider.points[a], firstPosition), firstRotate), secondRotate);
                                avg += tr[a].Z/8f;
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
                                debugCanvas.Children.Add(line);
                            }
                            index++;

                            if (collider.touchingColliders.Count > 0)
                            {
                                foreach (Collider touching in collider.touchingColliders)
                                {
                                    var r = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(touching.position, firstPosition), firstRotate), secondRotate));
                                    var a = RenderPoint(Vector3.Transform(Vector3.Transform(Vector3.Transform(collider.position, firstPosition), firstRotate), secondRotate));
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
                                }
                                index++;
                            }


                        }
                    }
                }
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
            for (int i = 0; i < points.Length; i++) {
                p.Add(new Point((int)((fov / (fov + points[i].Z)) * points[i].X) + (Width / 2), (int)((fov / (fov + points[i].Z)) * points[i].Y) + (Height / 2)));
            }
            FindCenter();
            visual = p.ToArray();
            Point all = new Point(0,0);
            foreach(Point point in visual)
            {
                all = new Point(all.X + point.X, all.Y + point.Y);
            }
            all = new Point(all.X/p.Count, all.Y/p.Count);
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
    public enum MovementType//
    {
        Frozen,
        Velocity,
        All
    }
    public enum HitReaction //empty untill i learn physics :( 
    {
        None,

    }
    public class Collider //will be used for raycasts
    {
        public bool hit { get; set; }
        public Vector3 position { get; set; }
        public Vector3 scale { get; set; }
        public Vector3 size { get; set; }
        public bool enabled { get; set; }
        public HitReaction hitReaction { get; set; }
        public MovementType movementType { get; set; }

        public Vector3 rotation { get; set; }
        public float gravityScale { get; set; }
        public float initalDistance { get; set; }//if the distance between two objects is greater than this it ignores the collison between

        public Vector3[] points { get; set; }

        public Object3d possibleParent { get; set; }
        public List<Collider> touchingColliders { get; set; }
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
                new Vector3(size.X, size.Y, size.Z) * scale + position,
                new Vector3(-size.X, size.Y, size.Z) * scale + position,
                new Vector3(-size.X, size.Y, -size.Z) * scale + position,
                new Vector3(size.X, size.Y, -size.Z) * scale + position,
                new Vector3(size.X, -size.Y, size.Z) * scale + position,
                new Vector3(-size.X, -size.Y, size.Z) * scale + position,
                new Vector3(-size.X, -size.Y, -size.Z) * scale + position,
                new Vector3(size.X, -size.Y, -size.Z) * scale + position
            };
            Matrix4x4 m = Matrix4x4.CreateFromYawPitchRoll(rotation.Y, rotation.X, rotation.Z);
            for (int i = 0; i < points.Length; i++)
            {
                Vector3 clone = points[i];
                points[i] = Vector3.TransformNormal(clone - position, m) * scale + position;
            }
            initalDistance = Vector3.Distance(position, points[0]);
        }
        public Collider(Vector3 position, Vector3 scale, Vector3 size, HitReaction hitReaction, MovementType movementType)
        {
            this.position = position;
            this.scale = scale;
            this.size = size;
            this.hitReaction = hitReaction;
            initalDistance = Vector3.Distance(position, new Vector3(size.X, size.Y, size.Z) * scale);
            this.movementType = movementType;
            hit = false;
            touchingColliders = new List<Collider>();
            
            
        }
    }
    public class Object3d //main folder for all infomation reguarding faces, collisions, rotaions and all that fun stuff
    {
        public Vector3[] basePoints { get; set; }
        public Vector3 rotation { get; set; }
        public Vector3 position { get; set; }
        public Vector3[] worldPoints { get; set; }
        public Vector3 center { get; set; }
        public Vector3 scale { get; set; }
        public List<Face> squareFaces { get; set; }
        public int[] orginalFaceData { get; set; }

        public Vector3 Velocity { get; set; }
        public float Weight { get; set; }
        public Collider[] colliders { get; set; }


        public void FixColliders()
        {
            if (colliders!= null) 
            foreach(Collider col in colliders)
            {
                col.rotation = rotation;
                col.scale = scale;
                col.position = position;
                col.GeneratePoints();
                col.possibleParent = this;
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
        public Object3d(Vector3[] basePoints1, int[] faces, Vector3 scale1)
        {
            //Vector3[] clone = basePoints1.Concatenate(new Vector3[] { new Vector3(1f, 1f, 1f) } );
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
            //Vector3[] clone = basePoints1.Concatenate(new Vector3[] { new Vector3(1f, 1f, 1f) } );
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


