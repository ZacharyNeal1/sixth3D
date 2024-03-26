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
using System.Windows.Shapes;

namespace sixth3D
{
    /// <summary>
    /// Interaction logic for Editor.xaml
    /// </summary>
    public partial class Editor : Window
    {
        Canvas Main = new Canvas();
        DataGrid grid = new DataGrid()
        {
        };
        DataGrid renderGrid = new DataGrid()
        {
        };
        List<Value> valList = new List<Value>()
            {
                new Value("app width"),
                new Value("app height"),
                new Value("cam nearplane"),
                //new Value("cam focaldist"),
                new Value("cam fov"),
                new Value("lookspeed"),
                new Value("cam2 pos"),
                new Value("cam2 rot"),
            };
        List<Value> renderValList = new List<Value>()
            {
                new Value("render cam forward to floor"),
                new Value("render cam forward"),
                new Value("render raycast plane normal"),
                new Value("render raycast plane xy"),
            };
        Button b = new Button();
        Button b1 = new Button();
        public Graph g = new Graph();
        public Editor()
        {
            InitializeComponent();

            this.Content = Main;





            g.output = Main;

            b.RenderTransform = new TranslateTransform() { X = 0f, Y = 0f };
            b.Width = 100f;
            b.Height = 50f;
            b.Click += Button_Click;
            b.Content = new TextBlock() { Text = "get values" };
            b1.Width = 100f;
            b1.Height = 50f;
            b1.Content = new TextBlock() { Text = "set values" };
            b1.RenderTransform = new TranslateTransform() { X = 100f, Y = 0f };
            b1.Click += Button_Click1;


            renderGrid.ItemsSource = renderValList;
            grid.ItemsSource = valList;
            Main.Children.Add(renderGrid);
            Main.Children.Add(grid);
            Main.Children.Add(b);
            Main.Children.Add(b1);

            grid.RenderTransform = new TranslateTransform() { X = 0, Y = 50 };
            grid.UpdateLayout();

            grid.Height = 250f;
            renderGrid.Height = 250f;
            renderGrid.RenderTransform = new TranslateTransform() { X = 250, Y = 50 };


        }
        public class Graph
        {
            public float[] values { get; set; } = new float[300];
            public float baselineValue { get; set; } = 50f;
            public float baseValue { get; set; } = 100f;
            public bool resize { get; set; } = true;
            public float baseX { get; set; } = 0f;
            public float baseY { get; set; } = 100f;
            public float canvasWidth { get; set; } =400f;
            public float canvasHeight { get; set; } = 100f;
            public Canvas output { get; set; }
            public int outputIndex { get; set; } = -1;
            public int outputIndex2 { get; set; } = -1;
            public Graph()
            {
                for (int i = 0; i < values.Length; i ++)
                {
                    values[i] = baseValue;
                }
            }
            public void AddValue( float value)
            {
                var final = new float[values.Length];
                for (int i = 1; i < values.Length; i ++)
                {
                    final[i - 1] = values[i];
                }
                final[values.Length - 1] = value;
                values = final;
                Make();
            }
            public void Make()
            {
                List<Point> points = new List<Point>(values.Length);
                float singleX = canvasWidth / values.Length;
                for (int i = 0; i < values.Length; i ++)
                {
                    points.Add(
                        new Point(
                            (singleX* i) + baseX,
                            (Math.Clamp(((values[i] / baseValue) - 0.5f),-0.5f,0.5f)* canvasHeight) + baseY + canvasHeight
                        )
                    );
                }

                if (outputIndex == -1)
                {
                    output.Children.Add(new Polyline());
                    outputIndex = output.Children.Count - 1;
                    output.Children.Add(new Polyline());
                    outputIndex2 = output.Children.Count - 1;
                }
                var a = output.Children[outputIndex] as Polyline;
                a.Visibility = Visibility.Visible;
                a.Points = new PointCollection(points.ToArray());
                a.Stroke = Brushes.Olive;
                a.Fill = Brushes.Transparent;
                a.StrokeThickness = 1f;

                var c = new Point[]
                {
                    new Point(baseX, baseY),
                    new Point(baseX + canvasWidth, baseY),
                    new Point(baseX + canvasWidth, baseY + canvasHeight),
                    new Point(baseX, baseY + canvasHeight)
                };

                var b = output.Children[outputIndex2] as Polyline;
                b.Points = new PointCollection(c);
                b.Stroke = Brushes.SlateGray;
                b.Fill = Brushes.Transparent;
                b.StrokeThickness = 2f;

            }
        }
        public class Value
        {
              public string varName { get; set; }
            public string value { get; set; }
            public bool valueB { get; set; }
            public Value(string n)
            {
                varName = n;
                value = "_";
            }
            public void Update()
            {
                var ap = Application.Current.MainWindow as MainWindow;
                if (ap == null) value = "cant fetch"; else
                    switch (varName)
                    {

                        case "app width":
                            value = ap.Width.ToString();
                             break;
                        case "app height":value = ap.Height.ToString();
                            break;
                        case "cam nearplane":
                            value = ap.cam.nearPlane.ToString();
                            break;
                        //case "cam focaldist":
                        //    value = ap.cam.focalDistance.ToString();
                        //    break;
                        case "cam fov":
                            value = ap.cam.fov.ToString();
                            break;
                        case "lookspeed":
                            value = ap.lookSpeed.ToString();
                            break;
                        case "cam2 pos":
                            value = ap.cam2.position.ToString();
                            break;
                        case "cam2 rot":
                            value = ap.cam2.rotation.ToString();
                            break;
                        case "render cam forward to floor":
                            valueB = ap.debugRenders[0];
                            break;
                        case "render cam forward":
                            valueB = ap.debugRenders[1];
                            break;
                        case "render cam raycast plane normal":
                            valueB = ap.debugRenders[2];
                            break;
                        case "render cam raycast plane xy":
                            valueB = ap.debugRenders[3];
                            break;
                    }

            }
            public void Set()
            {
                var ap = Application.Current.MainWindow as MainWindow;
                if (ap == null) value = "cant fetch";
                else
                    switch (varName)
                    {

                        case "app width":
                            ap.Width = double.Parse(value);
                            break;
                        case "app height":
                            ap.Height = double.Parse(value);
                            break;
                        case "cam nearplane":
                            ap.cam.nearPlane = float.Parse(value);
                            break;
                        //case "cam focaldist":
                        //    ap.cam.focalDistance = float.Parse(value);
                        //    break;
                        case "cam fov":
                            ap.cam.fov = float.Parse(value);
                            break;
                        case "lookspeed":
                            ap.lookSpeed = float.Parse(value);
                            break;
                        //case "cam2 pos":
                        //    ap.lookSpeed = Vector3.Parse(value);
                        //    break;
                        //case "cam2 rot":
                        //    ap.lookSpeed = Vector3.Parse(value);
                        //    break;
                        case "render cam forward to floor":
                            ap.debugRenders[0] = valueB;
                            break;
                        case "render cam forward":
                            ap.debugRenders[1] = valueB;
                            break;

                        case "render cam raycast plane normal":
                            ap.debugRenders[2] = valueB;
                            break;
                        case "render cam raycast plane xy":
                            ap.debugRenders[3] = valueB;
                            break;
                    }
            }

        }
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            foreach(Value v in valList)
            {
                v.Update();
            }
            foreach (Value v in renderValList)
            {
                v.Update();
            }
            InvalidateVisual();
        }
        private void Button_Click1(object sender, RoutedEventArgs e)
        {
            foreach (Value v in valList)
            {
                v.Set();
            }
            foreach (Value v in renderValList)
            {
                v.Set();
            }
            InvalidateVisual();
        }
    }
}
