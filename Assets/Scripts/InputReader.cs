using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;
using static PlayerInputActions;

namespace Kart
{
    [CreateAssetMenu(fileName = "InputReader", menuName = "Kart/InputReader")]
    public class InputReader : ScriptableObject, IPlayerActions, IDrive
    {
        private PlayerInputActions inputActions;
        public Vector2 Move => inputActions.Player.Move.ReadValue<Vector2>();
        public bool IsBraking => inputActions.Player.Brake.ReadValue<float>() > 0;

        private void OnEnable()
        {
            if (inputActions == null)
            {
                inputActions = new PlayerInputActions();
                inputActions.Player.SetCallbacks(this);
            }
        }
        
        public void Enable() {
            inputActions.Enable();
        }

        public void OnMove(InputAction.CallbackContext context)
        {
        }

        public void OnLook(InputAction.CallbackContext context)
        {
            
        }

        public void OnFire(InputAction.CallbackContext context)
        {
            
        }

        public void OnBrake(InputAction.CallbackContext context)
        {
            
        }
    }
}
