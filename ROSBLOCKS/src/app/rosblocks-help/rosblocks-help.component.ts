import { Component, ElementRef, ViewChild } from '@angular/core';

@Component({
  selector: 'app-rosblocks-help',
  templateUrl: './rosblocks-help.component.html',
  styleUrls: ['./rosblocks-help.component.css']
})
export class RosblocksHelpComponent {
  dropdowns: { [key: number]: boolean } = {};
  selectedVideo: string | null = null;

  onSearch(event: any): void {
    console.log('searching:', event.target.value);
  }

  toggleDropdown(id: number): void {
    this.dropdowns[id] = !this.dropdowns[id];
  }

  selectVideo(videoName: string): void {
    this.selectedVideo = videoName;
  }
}
