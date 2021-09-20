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

        public const int COL_NUM = 50 + 1;
        public const int ROW_NUM = 10000;

        public DataTable DataTable { private set; get; } = new DataTable();

        public DelegateCommand SaveCommand { get; }

        public DataView DataTableView => new DataView(DataTable);
        private void NotifyTableUpdate()
        {
            RaisePropertyChanged(nameof(DataTableView));
        }

        private void InitializeDataTable()
        {
            DataTable.Columns.Clear();
            DataTable.Rows.Clear();

            IEnumerable<int> col = Enumerable.Range('A', 'Z' - 'A' + 1);
            IEnumerable<int> row = Enumerable.Range(1, ROW_NUM);

            DataTable.Columns.Add("ID");

            int cc = 0;
            foreach (char c in col)
            {
                if (COL_NUM - 1 <= cc) { break; }

                DataTable.Columns.Add(c.ToString());

                ++cc;
            }
            foreach (char c1 in col)
            {
                foreach (char c2 in col)
                {
                    if (COL_NUM - 1 <= cc) { break; }

                    DataTable.Columns.Add(string.Format("{0}{1}", c1, c2));

                    ++cc;
                }
            }

            foreach (int r in row)
            {
                DataTable.Rows.Add(r.ToString());
            }

            using (var book = new XLWorkbook("test.xlsm", XLEventTracking.Disabled))
            {
                var sheet = book.Worksheet("しーと");

                for (int r = 0; r < ROW_NUM; ++r)
                {
                    for (int c = 1; c < COL_NUM; ++c)
                    {
                        string xlCellKey = DataTable.Columns[c].ColumnName + (r+1).ToString();
                        var xlCell = sheet.Cell(xlCellKey);
                        if (null != xlCell)
                        {
                            DataTable.Rows[r][c] = xlCell.Value.ToString();
                        }
                    }
                }
            }

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

                    for (int r = 0; r < DataTableViewModel.ROW_NUM; ++r)
                    {
                        for (int c = 1; c < DataTableViewModel.COL_NUM; ++c)
                        {
                            string xlCellKey = m_vm.DataTable.Columns[c].ColumnName + (r + 1).ToString();
                            var xlCell = sheet.Cell(xlCellKey);
                            if (null != xlCell)
                            {
                                xlCell.Value = m_vm.DataTable.Rows[r][c];
                            }
                        }
                    }

                    book.Save();
                }
            };
        }
    }
}