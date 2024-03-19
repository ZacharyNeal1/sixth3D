using System;
using System.Collections.Generic;
using System.Linq;
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
        List<Value> valList = new List<Value>()
            {
                new Value("app width"),
                new Value("app height"),
                new Value("cam nearplane"),
                //new Value("cam focaldist"),
                new Value("cam fov"),
                new Value("lookspeed"),
            };
        Button b = new Button();
        Button b1 = new Button();
        public Editor()
        {
            InitializeComponent();

            this.Content = Main;







            b.RenderTransform = new TranslateTransform() {X = 220f, Y= 100f };
            b.Width = 100f;
            b.Height = 100f;
            b.Click += Button_Click;
            b.Content = new TextBlock() { Text = "get values" };
            b1.Width = 100f;
            b1.Height = 100f;
            b1.Content = new TextBlock() { Text = "set values" };
            b1.RenderTransform = new TranslateTransform() { X = 220f, Y = 300f };
            b1.Click += Button_Click1;

            grid.ItemsSource = valList;
            Main.Children.Add(grid);
            Main.Children.Add(b);
            Main.Children.Add(b1);
        }

        public class Value
        {
              public string varName { get; set; }
            public string value { get; set; }
            public float valuef { get; set; }
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
                    }
            }

        }
        
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            foreach(Value v in valList)
            {
                v.Update();
            }
            grid.UpdateLayout();
            grid.ItemsSource = valList;
            grid.InvalidateVisual();
            grid.UpdateDefaultStyle();
        }
        private void Button_Click1(object sender, RoutedEventArgs e)
        {
            foreach (Value v in valList)
            {
                v.Set();
            }
            InvalidateVisual();
        }
    }
}
