   M S B A M L   `   `   `  ����T  PDiscreteGestureBasics-WPF, Version=1.0.0.0, Culture=neutral, PublicKeyToken=nullo<clr-namespace:Microsoft.Samples.Kinect.DiscreteGestureBasics.Microsoft.Samples.Kinect.DiscreteGestureBasics  5   	   ?    9Microsoft.Samples.Kinect.DiscreteGestureBasics.MainWindow   5   	   \ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   Nlocal<clr-namespace:Microsoft.Samples.Kinect.DiscreteGestureBasics     5   	   -   5          � Title$  Discrete Gesture Basics��5   	    � WindowStyle$
 None=�5   	    � WindowState$ 	Maximized=�5   	   $��902.75��5   &   $��1600.5��66    � 	Resources 5	         MediumGreyBrush �� Color  KinectPurpleBrush  KinectBlueBrush �� DataTypeF   @Microsoft.Samples.Kinect.DiscreteGestureBasics.GestureResultView � RowDefinitions �� StringFormat��%�  &	        &	 #     &	 F     (�� i     5      *, +)�� 5
   
   $ 	#ff6e6e6e��62   �� 5   
   $ 	#ff52318f��64   �� 5   
   $ 	#ff00BCF2��62   �� 5   
   "  6   .R�5      � $
��Auto��6   $��5q�6    ��6+   �� *	BodyColor+ 5      �� 5      $
��Auto �6$   �� 5      �� 5      $
��Auto �6$   �� 5      $
��Auto �6$   .+�5      �� $��Stretch=�6   $��14(�6u   $��Segoe UI)�6�   $��SemiBold"�6�   $��5q�6�   ��6:   �� *	BodyIndex+$ Body Index: {0}��6�   �� 5      $��Uniform=�67   $��1��6I   $��5q�6V   ��6   �� *ImageSource+6V   �� 5      $��18(�6R   $��Segoe UI)�6`   $��SemiBold"�6v   $��2��6�   $��5 5 0 0q�6�   ��6   �� *
Detected+$ Seated: {0}��6�   �� 5      $��18(�6X   $��Segoe UI)�6f   $��SemiBold"�6|   $��3��6�   $��5 0 0 0q�6�   ��6   �� *
Confidence+$ Confidence: {0}��6�   5      5      5      .��5      � $��	10 0 10 0q�6    � ColumnDefinitions 5    
   �� 5!       5#   
   �� 5$      $
��Auto �6   �� 5%      $��* �6   �� 5&      $
��Auto �6   .+�5)   
   (  "System.Windows.Controls.WebBrowser  -   5        $��BrowserInit��5)   "   $��1��6   $��-10, 0,-10, 0q�65   	   Source$3	 -http://antoine.kairos-agency.com/kinectintro/?�6L   �� 5+   
   -   5        $��finvideointro��5+   �   $��1��6   $��Images\cailloux_2.jpg��6   $
��Left=�6<   $
��Fill=�6W   $��-10,0q�6f   $��2��6u   $y�Hidden=�6�   �� 5-   
   -   5        $��
actionmain��5-   �   $��1��6   $��Images\porte_main.jpg��6   $
��Left=�6<   $
��Fill=�6W   $��-10,0q�6f   $��2��6u   $y�Hidden=�6�     5/   
   -   5        $��Browser1��5/   =   $��1��6   $
��Left=�6"   $��-10,0,-10,0q�6M   $y�Hidden=�6b   �� 51   
   -   5        $��fondDecran2��51   �   $��1��6   $!��Images\astronaute_image.jpg��6   $
��Left=�6B   $
��Fill=�6]   $��-10,0q�6l   $��2��6{   $y�Hidden=�6�   �� 52   
   -   5        $��
quizzImage��52   �   $��1��6   $��Images\quizz_00.jpg��6   $
��Left=�6:   $
��Fill=�6U   $��-10,0q�6d   $��2��6s   $y�Hidden=�6�   � 56   
   -   5        
 �Name$
 contentGrid��56      $��1��6$   $��2��61    57      �� 58      $��1* �6#   �� 59      $��2* �6#    5;      �� 5<      �� 5=      �� 5>      5B      �� 5C   
    �� Orientation$ Vertical=�6   $��2��6,   $��3��6<   $��
20 10 0 10q�6M   $��Center=�6a   $��Center=�6~   .+�5D      (� -	   5         (�Name$ kinectBodyViewbox��5D      .��5E      �� $��UniformToFill=�67   ��6   �� *ImageSource+67   5F      5G      �� 5I   
   -
   5        $��	statusBar��5I   ?   $��2��6   $��Stretch=�6!   $��Bottom=�6P   
�������#��[  6~   .@�5J      �� $��0��6;    J� RenderTransformOrigin$ 0.5,0.5&�6F   ��6   �� *
StatusText+z�5K      V� 5L      .~�5M      ��  �� ScaleY$ -1Q�6)   �� 5N      �� 5O      T� 5P      5Q      5S      5T      �� 5U   
   -   5        $��labelPosition��5U      $�� ��6%   
�������$
��Left=�6C   $��25,0,0,0q�6^   $��1��6p   $��Bottom=�6}   $	��673��6�   $ 0.457,0.538&�6�   5W      5X      