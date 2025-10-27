#!/usr/bin/env python3

"""
F1TENTH Localization Launch File
Uses Nav2 AMCL for particle filter based localization
"""

import os
import signal
import sys
import glob
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from PIL import Image, ImageTk
import threading
import queue
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


class MapSelectorGUI:
    def __init__(self):
        self.selected_map = None
        self.workspace_root = "/home/f1/f1tenth_ws"
        self.default_map = "/home/f1/f1tenth_ws/joon_path_generate/maps/gap_map_final.yaml"
        self.maps = self._find_all_maps()

    def _find_all_maps(self):
        """Find all yaml map files in the workspace"""
        map_patterns = [
            f"{self.workspace_root}/**/maps/*.yaml",
            f"{self.workspace_root}/maps/*.yaml",
            f"{self.workspace_root}/joon_path_generate/maps/*.yaml",
            f"{self.workspace_root}/path_generate/**/maps/*.yaml"
        ]

        all_maps = []
        for pattern in map_patterns:
            all_maps.extend(glob.glob(pattern, recursive=True))

        print(f"🔍 검색된 맵 파일들: {len(all_maps)}개")
        for map_file in all_maps:
            print(f"   - {map_file}")

        # Remove duplicates and sort
        unique_maps = sorted(list(set(all_maps)))

        # Create map info dictionary
        map_info = []
        for map_file in unique_maps:
            relative_path = os.path.relpath(map_file, self.workspace_root)
            map_name = os.path.basename(map_file).replace('.yaml', '')

            # Try to get map image if it exists
            map_image_path = map_file.replace('.yaml', '.pgm')
            if not os.path.exists(map_image_path):
                map_image_path = map_file.replace('.yaml', '.png')

            map_info.append({
                'name': map_name,
                'path': map_file,
                'relative_path': relative_path,
                'image_path': map_image_path if os.path.exists(map_image_path) else None
            })

        return map_info

    def show_map_selector(self):
        """Show GUI map selector window"""
        if not self.maps:
            messagebox.showerror("Error", "No map files found in the workspace!")
            return self.default_map

        # Create main window
        root = tk.Tk()
        root.title("F1TENTH Map Selector")
        root.geometry("800x600")
        root.resizable(True, True)

        # Configure grid weights for responsive design
        root.grid_rowconfigure(1, weight=1)
        root.grid_columnconfigure(0, weight=1)

        # Header frame
        header_frame = ttk.Frame(root)
        header_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=10)

        title_label = ttk.Label(
            header_frame,
            text="Select a Map for F1TENTH Localization"
        )
        title_label.grid(row=0, column=0, sticky="w")

        subtitle_label = ttk.Label(
            header_frame,
            text=f"Found {len(self.maps)} map files in workspace"
        )
        subtitle_label.grid(row=1, column=0, sticky="w", pady=(0, 5))

        # Main content frame
        main_frame = ttk.Frame(root)
        main_frame.grid(row=1, column=0, sticky="nsew", padx=10, pady=5)
        main_frame.grid_rowconfigure(0, weight=1)
        main_frame.grid_columnconfigure(0, weight=2)
        main_frame.grid_columnconfigure(1, weight=1)

        # Left panel - Map list
        left_frame = ttk.LabelFrame(main_frame, text="Available Maps", padding=10)
        left_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        left_frame.grid_rowconfigure(0, weight=1)
        left_frame.grid_columnconfigure(0, weight=1)

        # Treeview for map list
        columns = ('Name', 'Location')
        tree = ttk.Treeview(left_frame, columns=columns, show='tree headings', height=15)
        tree.grid(row=0, column=0, sticky="nsew")

        # Configure columns
        tree.heading('#0', text='#')
        tree.heading('Name', text='Map Name')
        tree.heading('Location', text='Path')

        tree.column('#0', width=40, minwidth=40)
        tree.column('Name', width=200, minwidth=150)
        tree.column('Location', width=300, minwidth=200)

        # Scrollbar for treeview
        scrollbar = ttk.Scrollbar(left_frame, orient="vertical", command=tree.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        tree.configure(yscrollcommand=scrollbar.set)

        # Populate treeview
        for i, map_info in enumerate(self.maps, 1):
            tree.insert('', 'end', text=str(i), values=(map_info['name'], map_info['relative_path']))

        # Right panel - Map preview
        right_frame = ttk.LabelFrame(main_frame, text="Map Preview", padding=10)
        right_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        right_frame.grid_rowconfigure(1, weight=1)
        right_frame.grid_columnconfigure(0, weight=1)

        preview_info_label = ttk.Label(right_frame, text="Select a map to see preview", foreground="gray")
        preview_info_label.grid(row=0, column=0, pady=5)

        preview_canvas = tk.Canvas(right_frame, bg="white", relief="sunken", borderwidth=2)
        preview_canvas.grid(row=1, column=0, sticky="nsew", pady=5)

        # Map details frame
        details_frame = ttk.Frame(right_frame)
        details_frame.grid(row=2, column=0, sticky="ew", pady=5)
        details_frame.grid_columnconfigure(1, weight=1)

        ttk.Label(details_frame, text="Selected:").grid(row=0, column=0, sticky="w")
        selected_label = ttk.Label(details_frame, text="None")
        selected_label.grid(row=0, column=1, sticky="w", padx=(10, 0))

        # Button frame
        button_frame = ttk.Frame(root)
        button_frame.grid(row=2, column=0, sticky="ew", padx=10, pady=10)

        # Buttons
        ttk.Button(
            button_frame,
            text="Browse Custom Map",
            command=lambda: self._browse_custom_map()
        ).pack(side="left", padx=(0, 10))

        ttk.Button(
            button_frame,
            text="Use Default Map",
            command=lambda: self._select_default_and_close(root)
        ).pack(side="left", padx=(0, 10))

        ttk.Button(
            button_frame,
            text="Cancel",
            command=lambda: self._cancel_and_close(root)
        ).pack(side="right", padx=(10, 0))

        ttk.Button(
            button_frame,
            text="Select Map",
            command=lambda: self._confirm_selection_and_close(root, tree)
        ).pack(side="right")

        # Event handlers
        def on_tree_select(event):
            selection = tree.selection()
            if selection:
                item = tree.item(selection[0])
                map_index = int(item['text']) - 1
                map_info = self.maps[map_index]

                selected_label.config(text=map_info['name'])
                preview_info_label.config(text=f"Map: {map_info['name']}")

                # Load and display map preview if available
                self._load_map_preview(preview_canvas, map_info)

        tree.bind('<<TreeviewSelect>>', on_tree_select)

        # Handle window close
        root.protocol("WM_DELETE_WINDOW", lambda: self._cancel_and_close(root))

        # Center window on screen
        root.update_idletasks()
        x = (root.winfo_screenwidth() - root.winfo_width()) // 2
        y = (root.winfo_screenheight() - root.winfo_height()) // 2
        root.geometry(f"+{x}+{y}")

        # Start GUI
        root.mainloop()

        return self.selected_map or self.default_map

    def _load_map_preview(self, canvas, map_info):
        """Load and display map preview image"""
        canvas.delete("all")

        if map_info['image_path']:
            try:
                # Load image
                image = Image.open(map_info['image_path'])

                # Resize to fit canvas
                canvas_width = canvas.winfo_width()
                canvas_height = canvas.winfo_height()

                if canvas_width > 1 and canvas_height > 1:  # Canvas is initialized
                    image.thumbnail((canvas_width - 20, canvas_height - 20), Image.Resampling.LANCZOS)

                    photo = ImageTk.PhotoImage(image)
                    canvas.create_image(
                        canvas_width // 2,
                        canvas_height // 2,
                        image=photo,
                        anchor="center"
                    )
                    canvas.image = photo  # Keep reference

            except Exception as e:
                canvas.create_text(
                    canvas.winfo_width() // 2,
                    canvas.winfo_height() // 2,
                    text=f"Preview not available\n{str(e)[:50]}...",
                    fill="gray",
                    anchor="center"
                )
        else:
            canvas.create_text(
                canvas.winfo_width() // 2,
                canvas.winfo_height() // 2,
                text="No preview image found\n(.pgm or .png)",
                fill="gray",
                anchor="center"
            )

    def _browse_custom_map(self):
        """Browse for custom map file"""
        file_path = filedialog.askopenfilename(
            title="Select Custom Map File",
            filetypes=[("YAML files", "*.yaml"), ("All files", "*.*")],
            initialdir=self.workspace_root
        )
        if file_path:
            self.selected_map = file_path

    def _select_default_and_close(self, root):
        """Select default map and close"""
        self.selected_map = self.default_map
        root.quit()
        root.destroy()

    def _cancel_and_close(self, root):
        """Cancel selection and close"""
        self.selected_map = self.default_map
        root.quit()
        root.destroy()

    def _confirm_selection_and_close(self, root, tree):
        """Confirm selected map and close"""
        selection = tree.selection()
        if selection:
            item = tree.item(selection[0])
            map_index = int(item['text']) - 1
            self.selected_map = self.maps[map_index]['path']
        else:
            messagebox.showwarning("No Selection", "Please select a map or use default.")
            return

        root.quit()
        root.destroy()


def select_map_file():
    """GUI-based map file selection function with terminal fallback"""
    # Check if we have a display available
    if not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
        print("🖥️  No display available - using terminal selection...")
        return _terminal_map_selection()

    try:
        selector = MapSelectorGUI()
        selected_map = selector.show_map_selector()
        print(f"✅ Selected map: {os.path.basename(selected_map)}")
        return selected_map
    except Exception as e:
        print(f"❌ GUI selection failed: {e}")
        print("🔄 Falling back to terminal selection...")
        return _terminal_map_selection()


def _terminal_map_selection():
    """Terminal-based map selection as fallback"""
    workspace_root = "/home/f1/f1tenth_ws"
    map_patterns = [
        f"{workspace_root}/**/maps/*.yaml",
        f"{workspace_root}/maps/*.yaml",
        f"{workspace_root}/joon_path_generate/maps/*.yaml",
        f"{workspace_root}/path_generate/**/maps/*.yaml"
    ]

    all_maps = []
    for pattern in map_patterns:
        all_maps.extend(glob.glob(pattern, recursive=True))

    unique_maps = sorted(list(set(all_maps)))

    if not unique_maps:
        print("❌ No map files found in the workspace!")
        return "/home/f1/f1tenth_ws/joon_path_generate/maps/gap_map_final.yaml"

    print("\n🗺️  Available Map Files:")
    print("=" * 70)

    for i, map_file in enumerate(unique_maps, 1):
        relative_path = os.path.relpath(map_file, workspace_root)
        map_name = os.path.basename(map_file).replace('.yaml', '')
        print(f"{i:2d}. {map_name:<25} → {relative_path}")

    print("=" * 70)

    while True:
        try:
            choice = input(f"\n🎯 Select map file (1-{len(unique_maps)}) or [ENTER] for default: ").strip()

            if choice == "":
                default_map = "/home/f1/f1tenth_ws/joon_path_generate/maps/gap_map_final.yaml"
                print(f"✅ Using default map: {os.path.basename(default_map)}")
                return default_map

            choice_num = int(choice)
            if 1 <= choice_num <= len(unique_maps):
                selected_map = unique_maps[choice_num - 1]
                print(f"✅ Selected map: {os.path.basename(selected_map)}")
                return selected_map
            else:
                print(f"❌ Please enter a number between 1 and {len(unique_maps)}")

        except ValueError:
            print("❌ Please enter a valid number")
        except KeyboardInterrupt:
            print("\n🛑 Selection cancelled. Using default map.")
            return "/home/f1/f1tenth_ws/joon_path_generate/maps/gap_map_final.yaml"


def generate_launch_description():
    pkg_share = get_package_share_directory("slam_nav")
    f1tenth_stack_share = get_package_share_directory("f1tenth_stack")

    # Interactive map selection (only if not provided as argument)
    selected_map_file = select_map_file()

    # Safe mode exit handler
    def signal_handler(sig, frame):
        print("\n[SAFE MODE] Received interrupt signal. Shutting down safely...")
        sys.exit(0)

    # Register signal handlers for safe exit
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time if true"
    )

    map_yaml_file_arg = DeclareLaunchArgument(
        "map_yaml_file",
        default_value=selected_map_file,
        description="Full path to map yaml file to load"
    )
    
    amcl_config_file_arg = DeclareLaunchArgument(
        "amcl_config_file",
        default_value=PathJoinSubstitution([pkg_share, "config", "amcl_config.yaml"]),
        description="Full path to AMCL configuration file"
    )
    
    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start lifecycle nodes"
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="/home/ircv7/RACE/real_ws/rviz2_config/localization_with_path.rviz",
        description="Path to RViz config file"
    )

    # Launch configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    amcl_config_file = LaunchConfiguration("amcl_config_file")
    autostart = LaunchConfiguration("autostart")
    rviz_config = LaunchConfiguration("rviz_config")

    # Include F1TENTH bringup (sensors, VESC, TF, etc.)
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([f1tenth_stack_share, "launch", "bringup_launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time
        }.items()
    )

    # Map server node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "yaml_filename": map_yaml_file
        }]
    )

    # AMCL (Adaptive Monte Carlo Localization) node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config_file,
            {"use_sim_time": use_sim_time}
        ]
    )

    # Lifecycle manager for localization nodes
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "node_names": ["map_server", "amcl"]
        }]
    )

    # RViz2 node with localization config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen"
    )
    
    # Shutdown event handler for safe mode
    shutdown_event_handler = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: print("[SAFE MODE] All nodes have been safely terminated.")
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_yaml_file_arg,
        amcl_config_file_arg,
        autostart_arg,
        rviz_config_arg,
        shutdown_event_handler,
        bringup_launch,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        rviz_node
    ])