using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Kart
{
    public interface IDrive {
        Vector2 Move { get; }
        bool IsBraking { get; }
        void Enable();
    }
}
