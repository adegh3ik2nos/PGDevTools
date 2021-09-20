using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Input;

namespace xl_fast.src
{
    public class DelegateCommand : ICommand
    {
        public event EventHandler CanExecuteChanged;
        public Action<object> ExecuteHandler;
        public Func<object, bool> CanExecuteHandler;

        public bool CanExecute(object parameter)
        {
            return (this.CanExecuteHandler == null) ? true : this.CanExecuteHandler(parameter);
        }

        public void Execute(object parameter)
        {
            this.ExecuteHandler?.Invoke(parameter);
        }

        public void RaiseCanExecuteChanged()
        {
            this.CanExecuteChanged(this, null);
        }
    }
}
