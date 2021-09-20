using ClosedXML.Excel;
using System;
using System.Collections.Generic;
using System.Data;
using System.Linq;
using System.Windows.Input;

namespace xl_fast.src
{
    internal class DataTableViewModel : BasePropertyChanged
    {
        public DataTableViewModel()
        {
            SaveCommand = new SaveCommand(this);

            InitializeDataTable();
        }

        public DataTable DataTable { private set; get; } = new DataTable();

        public DelegateCommand SaveCommand { get; }

        private DataRowView selectedRow;
        public DataRowView SelectedRow
        {
            get => selectedRow;
            set
            {
                selectedRow = value;
                RaisePropertyChanged(nameof(SelectedRow));
            }
        }

        public DataView DataTableView => new DataView(DataTable);
        private void NotifyTableUpdate()
        {
            RaisePropertyChanged(nameof(DataTableView));
        }

        private void InitializeDataTable()
        {
            DataTable.Columns.Clear();
            DataTable.Rows.Clear();

            int colNum = 50;
            int rowNum = 10000;

            IEnumerable<int> col = Enumerable.Range('A', 'Z' - 'A' + 1);
            IEnumerable<int> row = Enumerable.Range(1, rowNum);

            DataTable.Columns.Add("＼").ReadOnly = true;
            foreach (int r in row)
            {
                DataTable.Rows.Add(r.ToString());
            }

            int cc = 0;
            foreach (char c in col)
            {
                if(colNum <= cc) { break; }
                ++cc;

                DataTable.Columns.Add(c.ToString());
            }
            foreach (char c1 in col)
            {
                foreach (char c2 in col)
                {
                    if (colNum <= cc) { break; }
                    ++cc;

                    DataTable.Columns.Add(string.Format("{0}{1}", c1, c2));
                }
            }

            using (var book = new XLWorkbook("test.xlsm", XLEventTracking.Disabled))
            {
                var sheet = book.Worksheet("しーと");

                DataTable.Rows[0][1] = sheet.Cell("A1").Value;
            }

            NotifyTableUpdate();
        }

        public void AddRow(int id, string name)
        {
            var row = DataTable.NewRow();
            row[0] = id;
            row[1] = name;
            DataTable.Rows.Add(row);
            NotifyTableUpdate();
        }

        public void AddCol(string name)
        {
            DataTable.Columns.Add(name);
            NotifyTableUpdate();
        }
    }

    internal class SaveCommand : DelegateCommand
    {
        private readonly DataTableViewModel m_vm;

        public SaveCommand(DataTableViewModel vm)
        {
            m_vm = vm;

            ExecuteHandler = (param) =>
            {
                using (var book = new XLWorkbook("test.xlsm", XLEventTracking.Disabled))
                {
                    var sheet = book.Worksheet("しーと");
                    sheet.Cell("A1").Value = m_vm.DataTable.Rows[0][1];

                    book.Save();
                }
            };
        }
    }
}